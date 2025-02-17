using UnityEngine;

namespace Assets.TrackGeneration
{
    public static class TextureGenerator
    {
        public static Texture2D CreateStartLineTexture()
        {
            int textureSize = 256;
            int squareSize = 128;
            Texture2D texture = new Texture2D(textureSize, textureSize);

            for (int y = 0; y < textureSize; y++)
            {
                for (int x = 0; x < textureSize; x++)
                {
                    bool isAlternate = ((x / squareSize) + (y / squareSize)) % 2 == 0;
                    texture.SetPixel(x, y, isAlternate ? Color.white : Color.black);
                }
            }

            texture.Apply();
            texture.filterMode = FilterMode.Point;
            texture.wrapMode = TextureWrapMode.Repeat;
            return texture;
        }

        public static Texture2D CreateGridMarkerTexture()
        {
            int textureSize = 64;
            Texture2D texture = new Texture2D(textureSize, textureSize);
            Color gridColor = new Color(0.9f, 0.9f, 0.9f);

            for (int y = 0; y < textureSize; y++)
            {
                for (int x = 0; x < textureSize; x++)
                {
                    texture.SetPixel(x, y, gridColor);
                }
            }

            texture.Apply();
            texture.filterMode = FilterMode.Bilinear;
            return texture;
        }

        public static Material CreateWallTexture(Material wallMaterial)
        {
            Color color1 = Color.red;
            Color color2 = Color.white;

            Texture2D texture = new Texture2D(512, 512);
            for (int y = 0; y < texture.height; y++)
            {
                for (int x = 0; x < texture.width; x++)
                {
                    bool isFirstColor = (x / 256) % 2 == 0;
                    Color color = isFirstColor ? color1 : color2;
                    texture.SetPixel(x, y, color);
                }
            }

            texture.Apply();
            texture.wrapMode = TextureWrapMode.Repeat;
            wallMaterial.SetTexture("_BaseMap", texture);

            return wallMaterial;
        }
    }
}
