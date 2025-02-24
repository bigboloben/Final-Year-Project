using UnityEngine;
using UnityEngine.Rendering;
using UnityEngine.Rendering.Universal;
using UnityEngine.Rendering.RenderGraphModule;
using System.Collections.Generic;


namespace Assets.Shaders.OutlineRenderingFeature
{
    public class VibrationFeature : ScriptableRendererFeature
    {
        public RenderPassEvent injectionPoint = RenderPassEvent.BeforeRenderingPostProcessing;

        private Material vibrationMaterial;
        private Material compositeMaterial; // New material for compositing

        private VibrationPass vibrationPass;
        private CompositePass compositePass; // New pass for compositing

        [SerializeField] LayerMask layerMask;
        [SerializeField][Range(0, 1)] float vibrationIntensity = 1.0f;

        private class TextureTransferData : ContextItem
        {
            public TextureHandle vibrationTexture;

            public override void Reset()
            {
                vibrationTexture = TextureHandle.nullHandle;
            }
        }

        public override void Create()
        {
            var vibrationShader = Shader.Find("Shader Graphs/VibrationShader");
            if (vibrationShader == null)
                Debug.LogError("Failed to find Vibration shader!");

            // Use the default URP Blit shader
            var blitShader = Shader.Find("Hidden/Universal Render Pipeline/Blit");
            if (blitShader == null)
                Debug.LogError("Failed to find URP Blit shader!");

            vibrationMaterial = CoreUtils.CreateEngineMaterial(vibrationShader);
            compositeMaterial = CoreUtils.CreateEngineMaterial(blitShader);

            vibrationPass = new VibrationPass(layerMask);
            vibrationPass.renderPassEvent = injectionPoint;

            compositePass = new CompositePass();
            compositePass.renderPassEvent = injectionPoint + 1; // Execute right after vibration pass
        }

        public override void AddRenderPasses(ScriptableRenderer renderer, ref RenderingData renderingData)
        {
            if (renderingData.cameraData.cameraType == CameraType.Game ||
                renderingData.cameraData.cameraType == CameraType.SceneView)
            {
                vibrationPass.Setup(vibrationMaterial);
                compositePass.Setup(compositeMaterial, vibrationIntensity);

                renderer.EnqueuePass(vibrationPass);
                renderer.EnqueuePass(compositePass);
            }
        }

        public class VibrationPass : ScriptableRenderPass
        {
            private Material normalMaterial;
            private List<ShaderTagId> shaderTagIDs = new();
            private LayerMask layerMask;

            public VibrationPass(LayerMask layerMask)
            {
                this.layerMask = layerMask;
                shaderTagIDs.Add(new ShaderTagId("UniversalForwardOnly"));
                shaderTagIDs.Add(new ShaderTagId("UniversalForward"));
                shaderTagIDs.Add(new ShaderTagId("SRPDefaultUnlit"));
                shaderTagIDs.Add(new ShaderTagId("LightweightForward"));
                shaderTagIDs.Add(new ShaderTagId("ToonShader"));
            }

            public void Setup(Material material)
            {
                normalMaterial = material;
            }

            public override void RecordRenderGraph(RenderGraph renderGraph, ContextContainer frameData)
            {
                var resourceData = frameData.Get<UniversalResourceData>();
                if (resourceData.isActiveTargetBackBuffer)
                {
                    Debug.LogError("Skipping render pass. VibrationPass requires an intermediate ColorTexture, we can't use the BackBuffer as a texture input.");
                    return;
                }

                var source = resourceData.activeColorTexture;

                // Create our vibration target texture with same specs as source
                var destinationDesc = renderGraph.GetTextureDesc(source);
                destinationDesc.name = "_VibrationTexture";
                destinationDesc.clearBuffer = true;
                destinationDesc.clearColor = Color.clear;
                TextureHandle destination = renderGraph.CreateTexture(destinationDesc);

                using (var builder = renderGraph.AddRasterRenderPass<PassData>("Vibration Pass", out var passData))
                {
                    passData.destination = destination;
                    InitRendererList(frameData, ref passData, renderGraph);

                    // Ensure renderer list is valid before proceeding
                    if (passData.rendererList.IsValid())
                    {
                        builder.UseRendererList(passData.rendererList);
                        builder.SetRenderAttachment(destination, 0);

                        builder.SetRenderFunc((PassData data, RasterGraphContext context) =>
                        {
                            // Clear the render target with a transparent color
                            context.cmd.ClearRenderTarget(RTClearFlags.Color, Color.clear, 1, 0);

                            // Only draw if the renderer list is valid
                            if (data.rendererList.IsValid())
                            {
                                context.cmd.DrawRendererList(data.rendererList);
                            }
                            else
                            {
                                Debug.LogWarning("Invalid renderer list in Vibration Pass");
                            }
                        });
                    }
                    else
                    {
                        Debug.LogWarning("Failed to create valid renderer list for Vibration Pass");
                    }
                }

                // Store the vibration texture for the composite pass
                var customData = frameData.Create<TextureTransferData>();
                customData.vibrationTexture = destination;
            }

            void InitRendererList(ContextContainer frameData, ref PassData passData, RenderGraph renderGraph)
            {
                UniversalRenderingData universalRenderingData = frameData.Get<UniversalRenderingData>();
                UniversalCameraData universalCameraData = frameData.Get<UniversalCameraData>();
                UniversalLightData universalLightData = frameData.Get<UniversalLightData>();

                FilteringSettings filteringSettings = new FilteringSettings(RenderQueueRange.opaque, layerMask);
                DrawingSettings drawingSettings = RenderingUtils.CreateDrawingSettings(shaderTagIDs,
                    universalRenderingData, universalCameraData, universalLightData, SortingCriteria.CommonOpaque);
                drawingSettings.overrideMaterial = normalMaterial;

                var para = new RendererListParams(universalRenderingData.cullResults, drawingSettings, filteringSettings);
                passData.rendererList = renderGraph.CreateRendererList(para);
            }

            private class PassData
            {
                public TextureHandle destination;
                public RendererListHandle rendererList;
            }
        }

        public class CompositePass : ScriptableRenderPass
        {
            private Material blitMaterial;
            private float intensity;
            private RTHandle tempRTHandle;
            private new ProfilingSampler profilingSampler;

            public void Setup(Material material, float vibrationIntensity)
            {
                blitMaterial = material;
                intensity = vibrationIntensity;
                profilingSampler = new ProfilingSampler("Vibration Composite");
            }

            public override void RecordRenderGraph(RenderGraph renderGraph, ContextContainer frameData)
            {
                // Get the vibration texture from previous pass
                var vibrationData = frameData.Get<TextureTransferData>();
                if (vibrationData.vibrationTexture.Equals(TextureHandle.nullHandle))
                {
                    Debug.LogWarning("Vibration texture is null. Skipping composite pass.");
                    return; // Skip if vibration pass didn't execute
                }

                var resourceData = frameData.Get<UniversalResourceData>();
                var cameraData = frameData.Get<UniversalCameraData>();

                if (resourceData.isActiveTargetBackBuffer)
                {
                    Debug.LogError("Skipping composite pass. CompositePass requires an intermediate ColorTexture.");
                    return;
                }

                // Get the current active color target from resource data
                var cameraTexture = resourceData.activeColorTexture;

                // Set up the pass
                using (var builder = renderGraph.AddRenderPass<PassData>("Vibration Composite", out var passData))
                {
                    passData.cameraTexture = cameraTexture;
                    passData.vibrationTexture = vibrationData.vibrationTexture;
                    passData.blitMaterial = blitMaterial;
                    passData.intensity = intensity;
                    passData.profilingSampler = profilingSampler;

                    // Declare read dependencies
                    builder.ReadTexture(cameraTexture);
                    builder.ReadTexture(vibrationData.vibrationTexture);

                    // We'll write to the current color target
                    builder.UseColorBuffer(cameraTexture, 0);

                    builder.SetRenderFunc((PassData data, RenderGraphContext context) =>
                    {
                        // Use profiling sampler
                        CommandBuffer cmd = context.cmd;
                        using (new ProfilingScope(cmd, data.profilingSampler))
                        {
                            // Setup the URP Blit shader parameters
                            data.blitMaterial.SetTexture("_BlitTexture", data.vibrationTexture);

                            // Use Additive blending mode from the URP Blit shader
                            int blendMode = 1; // Additive blend

                            // Set blend intensity
                            data.blitMaterial.SetFloat("_BlitScaleBias", data.intensity);

                            // Draw fullscreen quad that blends vibration texture with camera texture
                            // Note: This writes to the active render target which is already set to camera texture
                            CoreUtils.DrawFullScreen(cmd, data.blitMaterial, shaderPassId: blendMode);
                        }
                    });
                }
            }

            private class PassData
            {
                public TextureHandle cameraTexture;
                public TextureHandle vibrationTexture;
                public Material blitMaterial;
                public float intensity;
                public ProfilingSampler profilingSampler;
            }
        }

        protected override void Dispose(bool disposing)
        {
            CoreUtils.Destroy(vibrationMaterial);
            CoreUtils.Destroy(compositeMaterial);
        }
    }
}