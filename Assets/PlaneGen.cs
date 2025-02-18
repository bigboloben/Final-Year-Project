using UnityEngine;
using UnityEditor;
using System.IO;

#if UNITY_EDITOR
public class CustomPlanePrefabCreator : MonoBehaviour
{
    [SerializeField] private int subdivisions = 10;
    [SerializeField] private float size = 10f;
    [SerializeField] private string prefabName = "CustomPlane";
    [SerializeField] private string savePath = "Assets/Prefabs";

    private Mesh generatedMesh;

    public void GeneratePlane()
    {
        // Create mesh object
        generatedMesh = new Mesh();
        generatedMesh.name = "CustomPlaneMesh";

        // Calculate vertices
        Vector3[] vertices = new Vector3[(subdivisions + 1) * (subdivisions + 1)];
        Vector2[] uv = new Vector2[vertices.Length];

        float halfSize = size * 0.5f;
        float stepSize = size / subdivisions;

        for (int i = 0; i <= subdivisions; i++)
        {
            for (int j = 0; j <= subdivisions; j++)
            {
                float x = -halfSize + (j * stepSize);
                float z = -halfSize + (i * stepSize);

                vertices[i * (subdivisions + 1) + j] = new Vector3(x, 0f, z);
                uv[i * (subdivisions + 1) + j] = new Vector2((float)j / subdivisions, (float)i / subdivisions);
            }
        }

        // Calculate triangles
        int[] triangles = new int[subdivisions * subdivisions * 6];
        int triangleIndex = 0;

        for (int i = 0; i < subdivisions; i++)
        {
            for (int j = 0; j < subdivisions; j++)
            {
                int vertexIndex = i * (subdivisions + 1) + j;

                triangles[triangleIndex] = vertexIndex;
                triangles[triangleIndex + 1] = vertexIndex + subdivisions + 1;
                triangles[triangleIndex + 2] = vertexIndex + 1;

                triangles[triangleIndex + 3] = vertexIndex + 1;
                triangles[triangleIndex + 4] = vertexIndex + subdivisions + 1;
                triangles[triangleIndex + 5] = vertexIndex + subdivisions + 2;

                triangleIndex += 6;
            }
        }

        // Calculate normals
        Vector3[] normals = new Vector3[vertices.Length];
        for (int i = 0; i < normals.Length; i++)
        {
            normals[i] = Vector3.up;
        }

        // Apply mesh data
        generatedMesh.vertices = vertices;
        generatedMesh.triangles = triangles;
        generatedMesh.normals = normals;
        generatedMesh.uv = uv;
    }

    public void CreatePrefab()
    {
        // Generate the mesh if it hasn't been generated
        if (generatedMesh == null)
        {
            GeneratePlane();
        }

        // Create the directory if it doesn't exist
        if (!Directory.Exists(savePath))
        {
            Directory.CreateDirectory(savePath);
        }

        // Save the mesh as an asset
        string meshPath = Path.Combine(savePath, prefabName + ".asset");
        AssetDatabase.CreateAsset(generatedMesh, meshPath);

        // Create a new GameObject with the mesh
        GameObject prefabObject = new GameObject(prefabName);
        MeshFilter meshFilter = prefabObject.AddComponent<MeshFilter>();
        meshFilter.sharedMesh = generatedMesh;
        prefabObject.AddComponent<MeshRenderer>();

        // Create the prefab
        string prefabPath = Path.Combine(savePath, prefabName + ".prefab");
        GameObject prefab = PrefabUtility.SaveAsPrefabAsset(prefabObject, prefabPath);

        // Clean up the temporary object
        DestroyImmediate(prefabObject);

        Debug.Log($"Prefab created at: {prefabPath}");

        // Refresh the asset database
        AssetDatabase.Refresh();
    }
}

[CustomEditor(typeof(CustomPlanePrefabCreator))]
public class CustomPlanePrefabCreatorEditor : UnityEditor.Editor
{
    public override void OnInspectorGUI()
    {
        DrawDefaultInspector();

        CustomPlanePrefabCreator creator = (CustomPlanePrefabCreator)target;

        if (GUILayout.Button("Generate Mesh"))
        {
            creator.GeneratePlane();
        }

        if (GUILayout.Button("Create Prefab"))
        {
            creator.CreatePrefab();
        }
    }
}
#endif