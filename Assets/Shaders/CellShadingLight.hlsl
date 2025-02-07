#ifndef CELL_SHADING_LIGHT
#define CELL_SHADING_LIGHT

#ifndef SHADERGRAPH_PREVIEW

struct EdgeConstants
{
    float edgeDiffuse;
    float edgeSpecular;
    float edgeSpecularOffset;
    float edgeDistanceAttenuation;
    float edgeShadowAttenuation;
    float edgeRim;
    float edgeRimOffset;
};

struct SurfaceVariables
{
    float3 normal;
    float3 view;
    float3 smoothness;
    float shininess;
    float rimThreshold;
    EdgeConstants ec;
};



float3 CalculateCellShading(Light light, SurfaceVariables surface)
{
    float shadowAtten = smoothstep(0.0f, surface.ec.edgeShadowAttenuation, light.shadowAttenuation);
    float distanceAtten = smoothstep(0.0f, surface.ec.edgeDistanceAttenuation, light.distanceAttenuation);
    float atten = shadowAtten * distanceAtten;
    
    float diffuse = saturate(dot(surface.normal, light.direction));
    diffuse *= atten;
    diffuse = smoothstep(0.0f, surface.ec.edgeDiffuse, diffuse);
    
    float3 halfDir = SafeNormalize(light.direction + surface.view);
    float specular = saturate(dot(surface.normal, halfDir));
    specular = pow(specular, surface.shininess);
    specular *= diffuse * surface.smoothness;
    specular = surface.smoothness * smoothstep((1-surface.smoothness) * surface.ec.edgeSpecular + surface.ec.edgeSpecularOffset, surface.ec.edgeSpecular + surface.ec.edgeSpecularOffset, specular);
    
    float rim = 1 - dot(surface.view, surface.normal);
    rim *= pow(diffuse, surface.rimThreshold);
    rim = surface.smoothness * smoothstep(surface.ec.edgeRim - surface.ec.edgeRimOffset, surface.ec.edgeRim + surface.ec.edgeRimOffset, rim);
    
    
    
    
    
    
    return light.color * (diffuse + max(specular, rim));
}
#endif

void CellShadingLight_float(float Smoothness, float RimThreshold, float3 Position, float3 Normal, float3 View, 
float EdgeDiffuse, float EdgeSpecular, float EdgeSpecularOffset, float EdgeDistanceAttenuation, float EdgeShadowAttenuation, float EdgeRim, float EdgeRimOffset,
out float3 Color)
{
    #if defined(SHADERGRAPH_PREVIEW)
        Color = float3(0.5f,0.5f,0.5f);
    #else
        SurfaceVariables surface;
        EdgeConstants ec;
        surface.normal = Normal;
        surface.view = SafeNormalize(View);
        surface.smoothness = Smoothness;
        surface.shininess = exp2(10 * Smoothness + 1);
        surface.rimThreshold = RimThreshold;
        ec.edgeDiffuse = EdgeDiffuse;
        ec.edgeSpecular = EdgeSpecular;
        ec.edgeSpecularOffset = EdgeSpecularOffset;
        ec.edgeDistanceAttenuation = EdgeDistanceAttenuation;
        ec.edgeShadowAttenuation = EdgeShadowAttenuation;
        ec.edgeRim = EdgeRim;
        ec.edgeRimOffset = EdgeRimOffset;
        surface.ec = ec;
    
        #if SHADOWS_SCREEN
            float4 clipPos = TransformWorldToHClip(Position);
            float4 shadowCoord = ComputeScreenPos(clipPos);
        #else
            float4 shadowCoord = TransformWorldToShadowCoord(Position);
        #endif
            Light light = GetMainLight(shadowCoord);
            Color = CalculateCellShading(light, surface);
    
            int pixelLightCount = GetAdditionalLightsCount();
            for (int i = 0; i < pixelLightCount; ++i)
            {
                light = GetAdditionalLight(i, Position, 1);
                Color += CalculateCellShading(light, surface);
            }
    #endif
    
    

}
#endif