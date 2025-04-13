#ifndef SOBEL_DEPTH_HLSL
#define SOBEL_DEPTH_HLSL

void SobelEdgeDetection_float(
    UnityTexture2D Texture, // Your source texture 
    UnitySamplerState Sampler, // Sampler for the texture
    float2 UV, // UV coordinates
    float2 TexelSize, // Size of one texel (typically 1/width, 1/height)
    float Strength, // Edge strength multiplier
    out float EdgeIntensity)   // Output edge intensity
{
    // Sample the 3×3 neighborhood
    float3 s00 = SAMPLE_TEXTURE2D(Texture, Sampler, UV + float2(-TexelSize.x, -TexelSize.y)).rgb;
    float3 s01 = SAMPLE_TEXTURE2D(Texture, Sampler, UV + float2(0, -TexelSize.y)).rgb;
    float3 s02 = SAMPLE_TEXTURE2D(Texture, Sampler, UV + float2(TexelSize.x, -TexelSize.y)).rgb;
    
    float3 s10 = SAMPLE_TEXTURE2D(Texture, Sampler, UV + float2(-TexelSize.x, 0)).rgb;
    float3 s11 = SAMPLE_TEXTURE2D(Texture, Sampler, UV).rgb; // Center pixel
    float3 s12 = SAMPLE_TEXTURE2D(Texture, Sampler, UV + float2(TexelSize.x, 0)).rgb;
    
    float3 s20 = SAMPLE_TEXTURE2D(Texture, Sampler, UV + float2(-TexelSize.x, TexelSize.y)).rgb;
    float3 s21 = SAMPLE_TEXTURE2D(Texture, Sampler, UV + float2(0, TexelSize.y)).rgb;
    float3 s22 = SAMPLE_TEXTURE2D(Texture, Sampler, UV + float2(TexelSize.x, TexelSize.y)).rgb;
    
    // Convert to luminance to focus on brightness edges
    float l00 = Luminance(s00);
    float l01 = Luminance(s01);
    float l02 = Luminance(s02);
    float l10 = Luminance(s10);
    float l11 = Luminance(s11);
    float l12 = Luminance(s12);
    float l20 = Luminance(s20);
    float l21 = Luminance(s21);
    float l22 = Luminance(s22);
    
    // Sobel kernels
    // Horizontal Sobel kernel: [-1, -2, -1; 0, 0, 0; 1, 2, 1]
    float gx = -l00 - 2.0 * l01 - l02 + l20 + 2.0 * l21 + l22;
    
    // Vertical Sobel kernel: [-1, 0, 1; -2, 0, 2; -1, 0, 1]
    float gy = -l00 - 2.0 * l10 - l20 + l02 + 2.0 * l12 + l22;
    
    // Calculate gradient magnitude
    EdgeIntensity = sqrt(gx * gx + gy * gy) * Strength;
}

// For depth-based edge detection
void SobelDepthEdge_float(
    UnityTexture2D DepthTexture, // Scene depth texture
    UnitySamplerState Sampler, // Sampler state
    float2 UV, // UV coordinates
    float2 TexelSize, // Texel size
    float DepthSensitivity, // Sensitivity to depth changes
    out float EdgeIntensity)      // Output edge intensity
{
    // Sample the 3×3 depth neighborhood
    float d00 = SAMPLE_TEXTURE2D(DepthTexture, Sampler, UV + float2(-TexelSize.x, -TexelSize.y)).r;
    float d01 = SAMPLE_TEXTURE2D(DepthTexture, Sampler, UV + float2(0, -TexelSize.y)).r;
    float d02 = SAMPLE_TEXTURE2D(DepthTexture, Sampler, UV + float2(TexelSize.x, -TexelSize.y)).r;
    
    float d10 = SAMPLE_TEXTURE2D(DepthTexture, Sampler, UV + float2(-TexelSize.x, 0)).r;
    float d11 = SAMPLE_TEXTURE2D(DepthTexture, Sampler, UV).r; // Center pixel
    float d12 = SAMPLE_TEXTURE2D(DepthTexture, Sampler, UV + float2(TexelSize.x, 0)).r;
    
    float d20 = SAMPLE_TEXTURE2D(DepthTexture, Sampler, UV + float2(-TexelSize.x, TexelSize.y)).r;
    float d21 = SAMPLE_TEXTURE2D(DepthTexture, Sampler, UV + float2(0, TexelSize.y)).r;
    float d22 = SAMPLE_TEXTURE2D(DepthTexture, Sampler, UV + float2(TexelSize.x, TexelSize.y)).r;
    
    // Apply Sobel operators to depth
    float depthGx = -d00 - 2.0 * d01 - d02 + d20 + 2.0 * d21 + d22;
    float depthGy = -d00 - 2.0 * d10 - d20 + d02 + 2.0 * d12 + d22;
    
    // Calculate depth edge intensity
    EdgeIntensity = sqrt(depthGx * depthGx + depthGy * depthGy) * DepthSensitivity;
}

// For normal-based edge detection
void SobelNormalEdge_float(
    UnityTexture2D NormalTexture, // Normal map
    UnitySamplerState Sampler, // Sampler state
    float2 UV, // UV coordinates
    float2 TexelSize, // Texel size
    float NormalSensitivity, // Sensitivity to normal changes
    out float EdgeIntensity)      // Output edge intensity
{
    // Sample the 3×3 normal neighborhood - normals are typically stored as RGB
    float3 n00 = SAMPLE_TEXTURE2D(NormalTexture, Sampler, UV + float2(-TexelSize.x, -TexelSize.y)).rgb * 2.0 - 1.0;
    float3 n01 = SAMPLE_TEXTURE2D(NormalTexture, Sampler, UV + float2(0, -TexelSize.y)).rgb * 2.0 - 1.0;
    float3 n02 = SAMPLE_TEXTURE2D(NormalTexture, Sampler, UV + float2(TexelSize.x, -TexelSize.y)).rgb * 2.0 - 1.0;
    
    float3 n10 = SAMPLE_TEXTURE2D(NormalTexture, Sampler, UV + float2(-TexelSize.x, 0)).rgb * 2.0 - 1.0;
    float3 n11 = SAMPLE_TEXTURE2D(NormalTexture, Sampler, UV).rgb * 2.0 - 1.0; // Center normal
    float3 n12 = SAMPLE_TEXTURE2D(NormalTexture, Sampler, UV + float2(TexelSize.x, 0)).rgb * 2.0 - 1.0;
    
    float3 n20 = SAMPLE_TEXTURE2D(NormalTexture, Sampler, UV + float2(-TexelSize.x, TexelSize.y)).rgb * 2.0 - 1.0;
    float3 n21 = SAMPLE_TEXTURE2D(NormalTexture, Sampler, UV + float2(0, TexelSize.y)).rgb * 2.0 - 1.0;
    float3 n22 = SAMPLE_TEXTURE2D(NormalTexture, Sampler, UV + float2(TexelSize.x, TexelSize.y)).rgb * 2.0 - 1.0;
    
    // Calculate differences in normal vectors
    float3 normalGx = -n00 - 2.0 * n01 - n02 + n20 + 2.0 * n21 + n22;
    float3 normalGy = -n00 - 2.0 * n10 - n20 + n02 + 2.0 * n12 + n22;
    
    // Calculate normal edge intensity
    EdgeIntensity = (length(normalGx) + length(normalGy)) * NormalSensitivity;
}

// Combined edge detection 
void SobelCombinedEdge_float(
    float ColorEdge, // Edge from color texture
    float DepthEdge, // Edge from depth texture
    float NormalEdge, // Edge from normal texture
    out float EdgeIntensity) // Final combined edge intensity
{
    // Simple maximum combination - you can adjust this to taste
    EdgeIntensity = max(max(ColorEdge, DepthEdge), NormalEdge);
    
    // Alternative weighted combination:
    // EdgeIntensity = ColorEdge * 0.3 + DepthEdge * 0.5 + NormalEdge * 0.2;
}