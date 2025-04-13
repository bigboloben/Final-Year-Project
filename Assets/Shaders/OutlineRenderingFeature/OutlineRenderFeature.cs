using UnityEngine;
using UnityEngine.Rendering;
using UnityEngine.Rendering.Universal;
using UnityEngine.Rendering.RenderGraphModule;
using UnityEngine.Rendering.RenderGraphModule.Util;
using UnityEditor.Rendering.Universal;
using System.Collections.Generic;
using Unity.VisualScripting;
using UnityEngine.Rendering.RendererUtils;

namespace Assets.Shaders.OutlineRenderingFeature
{
    public class OutlineRenderFeature : ScriptableRendererFeature
    {
        [System.Serializable]
        public class ShaderSettings
        {
            public float outlineScale = 1.0f;
            public float robertsCrossMultiplier = 1.0f;
            public float depthThreshold = 0.1f;
            [ColorUsage(true, true)]
            public Color outlineColor = Color.black;
            public float normalThreshold = 0.3f;
            public float steepAngleThreshold = 0.2f;
            public float steepAngleMultiplier = 2.0f;
        }

        public ShaderSettings shaderSettings = new ShaderSettings();
        public RenderPassEvent injectionPoint = RenderPassEvent.BeforeRenderingPostProcessing;

        //private NormalTexturesPass normalsPass;
        private Material normalMaterial;

        private NormalTexturePass normalsPass;


        private TransferMaterialPass transferPass;
        private Material outlinesMaterial;

        private const string k_NormalsTextureName = "_SceneViewSpaceNormals";
        [SerializeField] LayerMask layerMask;

        private class TextureTransferData : ContextItem
        {
            public TextureHandle normalsTexture;

            public override void Reset()
            {
                normalsTexture = TextureHandle.nullHandle;
            }
        }

        public override void Create()
        {
            var normalShader = Shader.Find("Shader Graphs/ViewSpaceNormal");
            var outlineShader = Shader.Find("Shader Graphs/OutlineShader");

            if (normalShader == null)
                Debug.LogError("Failed to find ViewSpaceNormals shader!");
            if (outlineShader == null)
                Debug.LogError("Failed to find Outline shader!");

            normalMaterial = CoreUtils.CreateEngineMaterial(normalShader);
            outlinesMaterial = CoreUtils.CreateEngineMaterial(outlineShader);

            ApplyShaderSettings(outlinesMaterial);

            //normalsPass = new NormalTexturesPass(layerMask);
            //normalsPass.renderPassEvent = injectionPoint;

            normalsPass = new NormalTexturePass(layerMask);
            normalsPass.renderPassEvent = injectionPoint;



            transferPass = new TransferMaterialPass();
            transferPass.renderPassEvent = injectionPoint;


        }

        private void ApplyShaderSettings(Material material)
        {
            if (material != null)
            {
                material.SetFloat("_OutlineScale", shaderSettings.outlineScale);
                material.SetFloat("_RobertsCrossMultiplier", shaderSettings.robertsCrossMultiplier);
                material.SetFloat("_DepthThreshold", shaderSettings.depthThreshold);
                material.SetColor("_OutlineColor", shaderSettings.outlineColor);
                material.SetFloat("_NormalThreshold", shaderSettings.normalThreshold);
                material.SetFloat("_SteepAngleThreshold", shaderSettings.steepAngleThreshold);
                material.SetFloat("_SteepAngleMultiplier", shaderSettings.steepAngleMultiplier);
            }
        }

        public override void AddRenderPasses(ScriptableRenderer renderer, ref RenderingData renderingData)
        {
            if (renderingData.cameraData.cameraType == CameraType.Game ||
                renderingData.cameraData.cameraType == CameraType.SceneView)
            //if (renderingData.cameraData.cameraType == CameraType.Game) 
            {
                normalsPass.Setup(normalMaterial);
                renderer.EnqueuePass(normalsPass);

                transferPass.Setup("_SceneViewSpaceNormals", outlinesMaterial);
                renderer.EnqueuePass(transferPass);

            }
        }

        public class NormalTexturePass : ScriptableRenderPass
        {
            private Material normalMaterial;
            private List<ShaderTagId> shaderTagIDs = new();
            private LayerMask layerMask;
            public NormalTexturePass(LayerMask layerMask)
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
                    Debug.LogError("Skipping render pass. NormalsPass requires an intermediate ColorTexture, we can't use the BackBuffer as a texture input.");
                    return;
                }
                var source = resourceData.activeColorTexture;
                var destinationDesc = renderGraph.GetTextureDesc(source);
                destinationDesc.name = "_SceneViewSpaceNormals";
                destinationDesc.clearBuffer = false;
                TextureHandle destination = renderGraph.CreateTexture(destinationDesc);

                using (var builder = renderGraph.AddRasterRenderPass<PassData>("Normals Pass", out var passData))
                {
                    InitRendererList(frameData, ref passData, renderGraph);
                    builder.UseRendererList(passData.rendererList);
                    builder.SetRenderAttachment(destination, 0);
                    builder.SetRenderFunc((PassData data, RasterGraphContext context) => { ExecutePass(data, context); });
                    builder.SetRenderFunc<PassData>(ExecutePass);
                }
                var customData = frameData.Create<TextureTransferData>();
                customData.normalsTexture = destination;
            }
            void ExecutePass(PassData data, RasterGraphContext context)
            {
                context.cmd.ClearRenderTarget(RTClearFlags.Color, Color.black, 1, 0);
                context.cmd.DrawRendererList(data.rendererList);
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

        //public class NormalTexturePass : ScriptableRenderPass
        //{
        //    const string name = "NormalsTexturePass";
        //    string normalTextureName;
        //    Material material;

        //    public NormalTexturePass()
        //    {
        //        requiresIntermediateTexture = true;
        //    }
        //    public void Setup(Material material)
        //    {
        //        //this.normalTextureName = normalTextureName;
        //        this.material = material;
        //    }

        //    public override void RecordRenderGraph(RenderGraph renderGraph, ContextContainer frameData)
        //    {
        //        var resourceData = frameData.Get<UniversalResourceData>();

        //        //TextureTransferData fetchedData = frameData.Get<TextureTransferData>();
        //        //var source = fetchedData.normalsTexture;

        //        var source = resourceData.activeColorTexture;

        //        var cameraTextureDesc = renderGraph.GetTextureDesc(source);
        //        cameraTextureDesc.name = $"_SceneViewSpaceNormals";
        //        cameraTextureDesc.clearBuffer = false;
        //        TextureHandle destination = renderGraph.CreateTexture(cameraTextureDesc);

        //        //if (!source.IsValid()) { return; }


        //        RenderGraphUtils.BlitMaterialParameters para = new(source, destination, material, 0);
        //        para.sourceTexturePropertyID = Shader.PropertyToID("_CameraColorTexture");
        //        renderGraph.AddBlitPass(para, passName: $"{name} _SceneViewSpaceNormalsBlit");

        //        var customData = frameData.Create<TextureTransferData>();
        //        customData.normalsTexture = destination;



        //    }
        //}
        public class TransferMaterialPass : ScriptableRenderPass
        {
            const string name = "TransferMaterialPass";
            string normalTextureName;
            Material material;

            public TransferMaterialPass()
            {
                requiresIntermediateTexture = true;
            }
            public void Setup(string normalTextureName, Material material)
            {
                this.normalTextureName = normalTextureName;
                this.material = material;
            }

            public override void RecordRenderGraph(RenderGraph renderGraph, ContextContainer frameData)
            {
                var resourceData = frameData.Get<UniversalResourceData>();

                TextureTransferData fetchedData = frameData.Get<TextureTransferData>();
                var source = fetchedData.normalsTexture;

                var cameraTH = resourceData.activeColorTexture;

                var cameraTextureDesc = renderGraph.GetTextureDesc(cameraTH);
                cameraTextureDesc.name = $"_CameraColorTexture";
                cameraTextureDesc.clearBuffer = false;
                TextureHandle cameraTexture = renderGraph.CreateTexture(cameraTextureDesc);

                if (!source.IsValid()) { return; }

                if (RenderGraphUtils.CanAddCopyPassMSAA())
                {
                    renderGraph.AddCopyPass(cameraTH, cameraTexture, passName: "Copy Camera Texture");
                    //renderGraph.AddCopyPass(cameraTexture, cameraTH, passName: name + "2");


                    RenderGraphUtils.BlitMaterialParameters para;
                    para = new(cameraTexture, resourceData.cameraColor, material, 0);
                    para.sourceTexturePropertyID = Shader.PropertyToID("_CameraColorTexture");
                    renderGraph.AddBlitPass(para, passName: $"Camera Texture Blit");

                    para = new(source, resourceData.cameraColor, material, 0);
                    para.sourceTexturePropertyID = Shader.PropertyToID(normalTextureName);
                    renderGraph.AddBlitPass(para, passName: $"Normal Texture Blit");
                }
                else { Debug.LogError("Cant add copy pass"); }

            }
        }


        protected override void Dispose(bool disposing)
        {
            CoreUtils.Destroy(normalMaterial);
            CoreUtils.Destroy(outlinesMaterial);
        }
    }
}