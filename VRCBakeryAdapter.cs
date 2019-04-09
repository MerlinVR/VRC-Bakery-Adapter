/**
 * MIT License
 * 
 * Copyright (c) 2019 Merlin
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

/**
 * A mostly self-contained script to handle replacing Bakery SH or RNM directional lightmap values so that VRC can support them.
 * This is so long mostly because I'm paranoid about making the process somewhat robust and difficult to break to a point where you lose data. The original script was like 1/3 the length it is now.
 * This script will go and automatically patch your Bakery shaders to support saving the directional lightmap properties into the material. This will not interfere with the usual behavior of Bakery.
 */ 

#if UNITY_EDITOR
using System;
using System.IO;
using System.Collections.Generic;
using System.Globalization;
using System.Reflection;
using System.Runtime.Serialization.Formatters.Binary;
using UnityEngine;
using UnityEditor;
using UnityEditor.SceneManagement;

[Serializable]
public class RendererMaterialList
{
    public MeshRenderer renderer;
    public Material[] materials;
};

// I'm sorry for these names, I made them when I was annoyed that I had to make them.
[Serializable]
public class SerializableRendererMaterialList
{
    public long rendererLocalId;
    public string[] materialAssetGUIDS;
}

[Serializable]
public class SerializableRendererMaterialLists
{
    public SerializableRendererMaterialList[] list;
}

public enum ReplacementScope
{
    Scene,
    Children,
};

[ExecuteInEditMode]
[AddComponentMenu("Merlin/VRC Bakery Adapter")]
public class VRCBakeryAdapter : MonoBehaviour
{
    private static readonly string savePath = "Assets/Bakery/generated/";
    public static readonly string revertProfilePath = "Bakery/reversionProfiles/";

    public RendererMaterialList[] OriginalRendererMaterials;
    public int LightmapMode = 3; // Default to SH
    public ReplacementScope replacementScope = ReplacementScope.Scene;
    public bool includeInactiveObjects = false;
    public string currentRevertPath = "";

    // Utils vars
    public bool showUtilsPane = false;
    public bool replaceTransparentStandard = false;

    void OnDestroy()
    {
        if (!EditorApplication.isPlayingOrWillChangePlaymode && !EditorApplication.isPlaying)
        {
            RevertMaterials();
        }
    }

    // https://forum.unity.com/threads/hash-function-for-game.452779/
    private static string ComputeMD5(string str)
    {
        System.Text.ASCIIEncoding encoding = new System.Text.ASCIIEncoding();
        byte[] bytes = encoding.GetBytes(str);
        var sha = new System.Security.Cryptography.MD5CryptoServiceProvider();
        return BitConverter.ToString(sha.ComputeHash(bytes)).Replace("-", "").ToLower();
    }

    public void RevertMaterials()
    {
        if (OriginalRendererMaterials != null)
        {
            for (int i = 0; i < OriginalRendererMaterials.Length; i++)
            {
                RendererMaterialList list = OriginalRendererMaterials[i];

                if (list != null && list.renderer != null && list.materials != null)
                {
                    list.renderer.sharedMaterials = list.materials;
                    OriginalRendererMaterials[i] = null; // Clear so we don't keep resetting the materials
                }
            }

            OriginalRendererMaterials = null;

            EditorSceneManager.MarkSceneDirty(gameObject.scene);

            // Clean up old revert data since we just effectively used it.
            if (currentRevertPath.Length > 0 && File.Exists(currentRevertPath))
            {
                File.Delete(currentRevertPath);
            }

            currentRevertPath = "";

            Debug.Log("Reverted Materials");
        }
    }

    private List<MeshRenderer> GetAllMeshRenderers()
    {
        List<MeshRenderer> rendererList = new List<MeshRenderer>();

        // Collect all mesh renderers in scope
        if (replacementScope == ReplacementScope.Scene)
        {
            GameObject[] rootObjects = gameObject.scene.GetRootGameObjects();

            foreach (GameObject gameObj in rootObjects)
            {
                rendererList.AddRange(gameObj.GetComponentsInChildren<MeshRenderer>(true));
            }
        }
        else
        {
            rendererList.AddRange(GetComponentsInChildren<MeshRenderer>(true));
        }

        return rendererList;
    }

    public void OnCollectMaterials()
    {
        // Make sure shaders have been patched
        PatchShaderProperties();

        List<MeshRenderer> rendererList = GetAllMeshRenderers();

        MaterialPropertyBlock propertyBlock = new MaterialPropertyBlock();

        // Revert materials before running again in case they haven't been cleaned up by the user
        RevertMaterials();

        OriginalRendererMaterials = new RendererMaterialList[rendererList.Count];
        Dictionary<string, Material> generatedMaterialList = new Dictionary<string, Material>();

        if (!Directory.Exists(savePath))
            Directory.CreateDirectory(savePath);

        for (int i = 0; i < rendererList.Count; i++)
        {
            EditorUtility.DisplayProgressBar("Conversion Progress", "Generating Bakery directional lightmap materials For VRChat...", i / (float)rendererList.Count);

            MeshRenderer meshRenderer = rendererList[i];

            bool skipRenderer = false;
            bool hasBakeryShader = false;

            foreach (Material mat in meshRenderer.sharedMaterials)
            {
                if (mat != null)
                {
                    string materialFileName = Path.GetFileNameWithoutExtension(AssetDatabase.GetAssetPath(mat));
                    if (materialFileName.StartsWith("generatedbakery_"))
                    {
                        // Prevent conflicts with other collectors that could make restoring materials a mess for the user.
                        skipRenderer = true;
                        break;
                    }

                    // Ignore renderers with no bakery shaders
                    if (mat.shader.name.Contains("Bakery"))
                        hasBakeryShader = true;
                }
            }

            if (!skipRenderer && hasBakeryShader && (meshRenderer.gameObject.activeInHierarchy || includeInactiveObjects))
            {
                meshRenderer.GetPropertyBlock(propertyBlock);

                // The property block will have these properties assigned by Bakery at runtime after directional lightmaps have been baked.
                // If the bakery scripts were whitelisted, they would assign these properties at runtime in-game, but for the meantime we have to do this manually.
                // The reason we need to generate new materials is that the mapping of property blocks and materials is not 1-to-1 you can have different property blocks set with different textures on 2 objects using the same material.
                Texture RNM0 = propertyBlock.GetTexture("_RNM0");
                Texture RNM1 = propertyBlock.GetTexture("_RNM1");
                Texture RNM2 = propertyBlock.GetTexture("_RNM2");

                if (RNM0 && RNM1 && RNM2)
                {
                    // Store original materials so we can restore them later
                    OriginalRendererMaterials[i] = new RendererMaterialList();
                    OriginalRendererMaterials[i].renderer = meshRenderer;
                    OriginalRendererMaterials[i].materials = meshRenderer.sharedMaterials;

                    string textureName = (AssetDatabase.GetAssetPath(RNM0) + "_" +
                        AssetDatabase.GetAssetPath(RNM1) + "_" +
                        AssetDatabase.GetAssetPath(RNM2));

                    Material[] newSharedMaterials = new Material[meshRenderer.sharedMaterials.Length];

                    for (int j = 0; j < meshRenderer.sharedMaterials.Length; j++)
                    {
                        Material material = meshRenderer.sharedMaterials[j];
                        string materialPath = AssetDatabase.GetAssetPath(material);
                        string materialFileName = Path.GetFileNameWithoutExtension(materialPath);

                        if (material != null && material.shader.name.Contains("Bakery"))
                        {
                            string matTexHash = ComputeMD5(materialPath + textureName);

                            Material newMaterial = null;
                            generatedMaterialList.TryGetValue(matTexHash, out newMaterial);

                            if (newMaterial == null)
                            {
                                newMaterial = new Material(material);
                                newMaterial.SetTexture("_RNM0", RNM0);
                                newMaterial.SetTexture("_RNM1", RNM1);
                                newMaterial.SetTexture("_RNM2", RNM2);
                                newMaterial.SetInt("bakeryLightmapMode", LightmapMode); // SH or RNM

                                AssetDatabase.CreateAsset(newMaterial, savePath + "generatedbakery_" + materialFileName + "_" + matTexHash + ".mat");

                                generatedMaterialList.Add(matTexHash, newMaterial);
                            }
                            else // We might have found an out of date material from an old bake so update its properties, this is mostly a sanity check and really shouldn't be needed
                            {
                                newMaterial.SetTexture("_RNM0", RNM0);
                                newMaterial.SetTexture("_RNM1", RNM1);
                                newMaterial.SetTexture("_RNM2", RNM2);
                                newMaterial.SetInt("bakeryLightmapMode", LightmapMode); // SH or RNM
                            }

                            newSharedMaterials[j] = newMaterial;
                        }
                        else if (material != null)
                        {
                            newSharedMaterials[j] = material;
                        }
                    }

                    meshRenderer.sharedMaterials = newSharedMaterials;
                }
                //else
                //{
                //    Debug.LogWarning("RNM/SH Textures not valid for " + meshRenderer.gameObject.name);
                //}
            }
        }

        EditorUtility.ClearProgressBar();

        AssetDatabase.SaveAssets();
        EditorSceneManager.MarkSceneDirty(gameObject.scene);

        // Backup revert settings
        SaveReversionProfile();

        // Be extra sure the materials are saved and such before the upload comes around.
        AssetDatabase.SaveAssets();
        EditorSceneManager.MarkSceneDirty(gameObject.scene);
        AssetDatabase.SaveAssets();

        Debug.Log("Converted Bakery materials for VRChat");
    }

    // Just a debug thing if people want to see a list of the renderers affected for some reason.
    public void OnPrintMaterials()
    {
        foreach (var rendererMat in OriginalRendererMaterials)
        {
            if (rendererMat != null && rendererMat.renderer != null && rendererMat.materials != null)
                Debug.Log(rendererMat.renderer.ToString() + ": " + rendererMat.materials.Length);
        }
    }

    // Inserts extra properties into the Properties block of the Bakery shaders so that they get serialized along with the materials
    // The references in the shaders reference these properties and assume they are bound via Property Blocks
    // This doesn't cause any interference with the usual behavior of the Bakery shaders since the Property Blocks take priority over these shader properties in editor.
    private bool PatchShader(string shaderPath)
    {
        const string patchString =
            "\r\n        // Merlin Patch\r\n" +
            "        _RNM0(\"RNM0\", 2D) = \"black\" {}\r\n" +
            "        _RNM1(\"RNM1\", 2D) = \"black\" {}\r\n" +
            "        _RNM2(\"RNM2\", 2D) = \"black\" {}\r\n" +
            "        [Enum(DEFAULT, 0, VERTEXLM, 1, RNM, 2, SH, 3)] bakeryLightmapMode(\"Bakery Lightmap Mode\", Int) = 0\r\n" +
            "        // End Merlin patch\r\n\r\n        ";

        string shaderText = "";

        try
        {
            using (StreamReader shaderReader = new StreamReader(shaderPath))
            {
                shaderText = shaderReader.ReadToEnd();
            }
        }
        catch (IOException)
        {
            Debug.LogError("Failed to open Bakery shader to patch!");
            return false;
        }

        if (shaderText.Contains("// Merlin Patch"))
        {
            Debug.Log("Bakery shader " + Path.GetFileNameWithoutExtension(shaderPath) + " has already been patched");
            return true;
        }

        // Look for the beginning of the property list
        int insertIdx = shaderText.IndexOf("_Color(\"Color\", Color)");

        if (insertIdx < 0)
        {
            Debug.LogError("Failed to find insertion point for shader properties");
            return false;
        }

        shaderText = shaderText.Insert(insertIdx, patchString);

        // Now write patched file out
        try
        {
            using (StreamWriter shaderWriter = new StreamWriter(shaderPath))
            {
                shaderWriter.Write(shaderText);
            }
        }
        catch (IOException)
        {
            Debug.LogError("Failed to save patch to Bakery shader");
            return false;
        }

        return true;
    }

    private void PatchShaderProperties()
    {
        const string standardPath = "Assets/Bakery/shader/BakeryStandard.shader";
        const string standardSpecPath = "Assets/Bakery/shader/BakeryStandardSpecular.shader";

        bool patchedStandard = PatchShader(standardPath);
        bool patchedStandardSpec = PatchShader(standardSpecPath);
        
        if (patchedStandard && patchedStandardSpec)
        {
            Debug.Log("Successfully patched Bakery shaders.");
        }
    }

    private void SaveReversionProfile()
    {
        string fileName = gameObject.scene.name + "_" + gameObject.name + "_" + DateTime.Now.ToString("dd-mm-yy_HH-mm-ss", CultureInfo.InvariantCulture) + ".rev";
        string savePath = Application.dataPath + "/" + revertProfilePath;
        //string savePath = revertProfilePath;
        string filePath = savePath + fileName;

        if (!Directory.Exists(savePath))
            Directory.CreateDirectory(savePath);

        // Beware all ye who read the following code, Unity asset serialization is terrible.
        // Soooo, we need to briefly re-implement the weak referencing for Mesh Renderers and Materials all over again here because Unity doesn't like serializing the mesh renderer references and just silently nulls them out like any respectable engine would do.
        // If there is some poorly-documented -- proper way to do this that you can demonstrate working in this case, please submit a pull request to put this poor code out of its misery. Thanks!

        // Yay reflection https://forum.unity.com/threads/how-to-get-the-local-identifier-in-file-for-scene-objects.265686/
        // This is somewhat broken on prefabs in >5.x. Objects under the root of the prefab will not have a local file ID because Unity just saves the prefab references into the scene now. 
        // Though with testing it seems to work on most prefabs I can try for some reason...
        // This sounds reasonable, except they left no decent way to uniquely identify components in prefabs now without dumb workarounds.
        // There's a proper solution if we were using >=2018.1 https://docs.unity3d.com/ScriptReference/AssetDatabase.TryGetGUIDAndLocalFileIdentifier.html
        PropertyInfo inspectorModeInfo = typeof(SerializedObject).GetProperty("inspectorMode", BindingFlags.NonPublic | BindingFlags.Instance);

        List<SerializableRendererMaterialList> serializableLists = new List<SerializableRendererMaterialList>();

        foreach (RendererMaterialList list in OriginalRendererMaterials)
        {
            if (list != null && list.renderer != null && list.materials != null)
            {
                SerializableRendererMaterialList newRendererMaterials = new SerializableRendererMaterialList();
                newRendererMaterials.materialAssetGUIDS = new string[list.materials.Length];

                for (int i = 0; i < list.materials.Length; i++)
                {
                    Material mat = list.materials[i];

                    if (mat != null)
                    {
                        newRendererMaterials.materialAssetGUIDS[i] = AssetDatabase.AssetPathToGUID(AssetDatabase.GetAssetPath(mat));
                    }
                    else
                    {
                        newRendererMaterials.materialAssetGUIDS[i] = "";
                    }
                }

                SerializedObject serializedObject = new SerializedObject(list.renderer);
                inspectorModeInfo.SetValue(serializedObject, InspectorMode.Debug, null);
                SerializedProperty localIdProperty = serializedObject.FindProperty("m_LocalIdentfierInFile"); // Misspelled purposely

                newRendererMaterials.rendererLocalId = localIdProperty.longValue;

                if (newRendererMaterials.rendererLocalId == 0)
                    Debug.Log("ID: " + newRendererMaterials.rendererLocalId + ", name: " + list.renderer.gameObject.name);

                if (newRendererMaterials.rendererLocalId != 0)
                    serializableLists.Add(newRendererMaterials);

                //inspectorModeInfo.SetValue(serializedObject, InspectorMode.Normal, null); // Restore normal inspector mode
            }
        }

        SerializableRendererMaterialLists serializableRendererMaterialLists = new SerializableRendererMaterialLists();
        serializableRendererMaterialLists.list = serializableLists.ToArray();

        // Now actually serialize the file as binary...
        BinaryFormatter formatter = new BinaryFormatter();

        using (FileStream dataFile = File.Create(filePath))
        {
            formatter.Serialize(dataFile, serializableRendererMaterialLists);
        }

        currentRevertPath = filePath;
    }

    public bool LoadReversionProfile(string path)
    {
        SerializableRendererMaterialLists lists;

        try
        {
            using (FileStream file = File.Open(path, FileMode.Open))
            {
                BinaryFormatter formatter = new BinaryFormatter();

                lists = (SerializableRendererMaterialLists)formatter.Deserialize(file);
            }
        }
        catch (IOException)
        {
            Debug.LogError("Failed to read reversion profile");
            return false;
        }

        PropertyInfo inspectorModeInfo = typeof(SerializedObject).GetProperty("inspectorMode", BindingFlags.NonPublic | BindingFlags.Instance);

        Dictionary<long, MeshRenderer> meshIdMap = new Dictionary<long, MeshRenderer>();
        GameObject[] sceneRootObjects = gameObject.scene.GetRootGameObjects();

        foreach (GameObject rootObj in sceneRootObjects)
        {
            MeshRenderer[] meshRenderers = rootObj.GetComponentsInChildren<MeshRenderer>();

            foreach (MeshRenderer sceneMeshRenderer in meshRenderers)
            {
                // Use same method as above to build a map of all mesh renderers in the scene to their IDs so we can easily remap them below.
                SerializedObject serializedObject = new SerializedObject(sceneMeshRenderer);
                inspectorModeInfo.SetValue(serializedObject, InspectorMode.Debug, null);
                SerializedProperty localIdProperty = serializedObject.FindProperty("m_LocalIdentfierInFile"); // Misspelled purposely

                long rendererId = localIdProperty.longValue;

                if (rendererId != 0)
                {
                    if (!meshIdMap.ContainsKey(rendererId))
                    {
                        meshIdMap.Add(rendererId, sceneMeshRenderer);
                    }
                    else
                    {
                        Debug.LogError("Duplicate ID " + rendererId + " found for renderer on " + sceneMeshRenderer.gameObject.name + ", belongs to renderer on " + meshIdMap[rendererId].gameObject.name);
                    }
                }
            }
        }

        List<RendererMaterialList> loadedRendererMaterialLists = new List<RendererMaterialList>();

        // Remap the IDs and GUIDs back to asset references now.
        foreach (SerializableRendererMaterialList list in lists.list)
        {
            RendererMaterialList newList = new RendererMaterialList();
            newList.materials = new Material[list.materialAssetGUIDS.Length];

            if (!meshIdMap.TryGetValue(list.rendererLocalId, out newList.renderer))
            {
                Debug.LogWarning("Unable to locate renderer for ID " + list.rendererLocalId);
                continue;
            }

            for (int i = 0; i < list.materialAssetGUIDS.Length; i++)
            {
                string matGuid = list.materialAssetGUIDS[i];
                Material loadedMat = null;

                if (matGuid.Length > 0)
                {
                    loadedMat = AssetDatabase.LoadAssetAtPath<Material>(AssetDatabase.GUIDToAssetPath(matGuid));

                    if (loadedMat == null)
                    {
                        Debug.LogWarning("Unable to load material with GUID " + matGuid);
                    }
                }

                newList.materials[i] = loadedMat;
            }

            loadedRendererMaterialLists.Add(newList);
        }

        OriginalRendererMaterials = loadedRendererMaterialLists.ToArray();

        return true;
    }

    public void ConvertStandardToBakeryStandard(bool useLightmapSpecular)
    {
        Shader bakeryStandard = Shader.Find("Bakery/Standard");
        Shader bakeryStandardSpec = Shader.Find("Bakery/Standard Specular");

        if (!bakeryStandard || !bakeryStandardSpec)
        {
            Debug.LogError("Could not locate Bakery shaders");
            return;
        }

        List<MeshRenderer> meshRenderers = GetAllMeshRenderers();
        HashSet<Material> materialList = new HashSet<Material>();

        foreach (MeshRenderer currentRenderer in meshRenderers)
        {
            foreach (Material currentMaterial in currentRenderer.sharedMaterials)
            {
                materialList.Add(currentMaterial);
            }
        }

        Material[] matArray = new Material[materialList.Count];
        int idx = 0;
        foreach (Material mat in materialList)
        {
            matArray[idx] = mat;
            idx++;
        }

        // Make this operation undo-able
        Undo.RecordObjects(matArray, "Standard -> Bakery Standard");

        foreach (Material currentMaterial in matArray)
        {
            if (currentMaterial)
            {
                string shaderName = currentMaterial.shader.name;
                bool isTransparentMaterial = currentMaterial.GetTag("RenderType", false) == "Transparent";

                if (!isTransparentMaterial || replaceTransparentStandard)
                {
                    if (shaderName == "Standard")
                    {
                        currentMaterial.shader = bakeryStandard;
                    }
                    else if (shaderName == "Standard (Specular setup)")
                    {
                        currentMaterial.shader = bakeryStandardSpec;
                    }

                    // Enable SH or RNM depending on the directional setting
                    if (LightmapMode == 3)
                    {
                        currentMaterial.SetInt("_BAKERY_SH", 1);
                        currentMaterial.EnableKeyword("BAKERY_SH");
                        currentMaterial.EnableKeyword("BAKERY_SHNONLINEAR");
                    }
                    else if (LightmapMode == 2)
                    {
                        currentMaterial.SetInt("_BAKERY_RNM", 1);
                        currentMaterial.EnableKeyword("BAKERY_RNM");
                    }

                    if (useLightmapSpecular)
                    {
                        currentMaterial.SetInt("_BAKERY_LMSPEC", 1);
                        currentMaterial.EnableKeyword("BAKERY_LMSPEC");
                    }
                }
            }
        }

        EditorSceneManager.MarkSceneDirty(gameObject.scene);
    }

    public void ConvertBakeryStandardToStandard()
    {
        Shader standard = Shader.Find("Standard");
        Shader standardSpec = Shader.Find("Standard (Specular setup)");

        List<MeshRenderer> meshRenderers = GetAllMeshRenderers();
        HashSet<Material> materialList = new HashSet<Material>();

        foreach (MeshRenderer currentRenderer in meshRenderers)
        {
            foreach (Material currentMaterial in currentRenderer.sharedMaterials)
            {
                materialList.Add(currentMaterial);
            }
        }

        Material[] matArray = new Material[materialList.Count];
        int idx = 0;
        foreach (Material mat in materialList)
        {
            matArray[idx] = mat;
            idx++;
        }

        // Make this operation undo-able
        Undo.RecordObjects(matArray, "Bakery Standard -> Standard");

        foreach (Material currentMaterial in matArray)
        {
            if (currentMaterial)
            {
                currentMaterial.SetInt("_BAKERY_SH", 0);
                currentMaterial.DisableKeyword("BAKERY_SH");

                currentMaterial.SetInt("_BAKERY_RNM", 0);
                currentMaterial.DisableKeyword("BAKERY_RNM");

                currentMaterial.SetInt("_BAKERY_LMSPEC", 0);
                currentMaterial.DisableKeyword("BAKERY_LMSPEC");

                currentMaterial.DisableKeyword("BAKERY_SHNONLINEAR");

                string shaderName = currentMaterial.shader.name;

                if (shaderName == "Bakery/Standard")
                {
                    currentMaterial.shader = standard;
                }
                else if (shaderName == "Bakery/Standard Specular")
                {
                    currentMaterial.shader = standardSpec;
                }
            }
        }

        EditorSceneManager.MarkSceneDirty(gameObject.scene);
    }
}

[CustomEditor(typeof(VRCBakeryAdapter))]
public class VRCBakeryAdapterInspector : Editor
{
    private static GUIContent replacementScopeLabel = new GUIContent("Replacement Scope", "Where to search for materials to replace. Scene scope will collect all materials in the scene this object belongs to. Child scope will collect all materials part of children gameobjects of this object.");
    private static GUIContent directionalModeLabel = new GUIContent("Lightmap Directional Mode", "The directional mode that the Bakery lightmaps have been baked with. If you aren't using either of these options, this adapter is not necessary.");
    private static GUIContent inactiveObjectsLabel = new GUIContent("Convert Inactive Objects", "If enabled, this will also modify the materials on disabled objects.");

    VRCBakeryAdapter adapter = null;
    private static GUIContent[] lightmapDirectionalityModes =
    {
        new GUIContent("RNM"),
        new GUIContent("SH"),
    };

    private static int[] lightmapDirectionalityModeValues =
    {
        2,
        3,
    };

    public void OnEnable()
    {
        adapter = (VRCBakeryAdapter)target;
    }

    public override void OnInspectorGUI()
    {
        EditorGUI.BeginDisabledGroup(EditorApplication.isPlayingOrWillChangePlaymode || EditorApplication.isPlaying); // Don't allow user to mess with settings in play mode

        EditorGUILayout.Space();

        EditorGUI.BeginChangeCheck();

        adapter.replacementScope = (ReplacementScope)EditorGUILayout.EnumPopup(replacementScopeLabel, adapter.replacementScope);
        adapter.LightmapMode = EditorGUILayout.IntPopup(directionalModeLabel, adapter.LightmapMode, lightmapDirectionalityModes, lightmapDirectionalityModeValues);
        adapter.includeInactiveObjects = EditorGUILayout.Toggle(inactiveObjectsLabel, adapter.includeInactiveObjects);

        if (EditorGUI.EndChangeCheck())
        {
            // Unity won't actually allow you to save the scene with Ctrl+S unless it's marked dirty
            EditorSceneManager.MarkSceneDirty(adapter.gameObject.scene);
        }

        EditorGUILayout.Space();

        if (GUILayout.Button("Run Conversion", GUILayout.Height(35)))
        {
            adapter.OnCollectMaterials();
        }
        
        bool replacerMaterialsInvalid = adapter == null || adapter.OriginalRendererMaterials == null || adapter.OriginalRendererMaterials.Length == 0;

        EditorGUI.BeginDisabledGroup(replacerMaterialsInvalid);
        if (GUILayout.Button("Revert Materials", GUILayout.Height(35)))
        {
            adapter.RevertMaterials();
        }
        EditorGUI.EndDisabledGroup();

        EditorGUILayout.Space();

        int numReplacedMaterials = 0;
        int rendererCount = 0;

        if (adapter.OriginalRendererMaterials != null && adapter.OriginalRendererMaterials.Length > 0)
        {
            // This probably shouldn't be run every GUI draw, but I'm lazy and it shouldn't take a crazy amount of time unless you are replacing tens of thousands of materials..
            foreach (RendererMaterialList list in adapter.OriginalRendererMaterials)
            {
                if (list != null && list.materials != null)
                {
                    rendererCount++;
                    foreach (Material mat in list.materials)
                    {
                        if (mat && mat.shader.name.Contains("Bakery"))
                            numReplacedMaterials++;
                    }
                }
            }

            if (numReplacedMaterials > 0)
                EditorGUILayout.HelpBox("This adapter has " + numReplacedMaterials + " replaced materials, revert your materials before editing them any further!", MessageType.Warning);
        }

        if (adapter.showUtilsPane = EditorGUILayout.Foldout(adapter.showUtilsPane, "Utilities"))
        {
            // Count unique materials for more in-depth stats
            HashSet<Material> materialSet = new HashSet<Material>();

            int totalUniqueMaterials = 0;
            int totalUniqueOriginalMaterials = 0;

            if (numReplacedMaterials > 0)
            {
                foreach (RendererMaterialList list in adapter.OriginalRendererMaterials)
                {
                    if (list != null && list.materials != null)
                    {
                        foreach (Material mat in list.materials)
                        {
                            if (mat != null && mat.shader.name.Contains("Bakery"))
                                materialSet.Add(mat);
                        }
                    }
                }

                totalUniqueOriginalMaterials = materialSet.Count;

                // Clear to use again
                materialSet.Clear();

                foreach (RendererMaterialList list in adapter.OriginalRendererMaterials)
                {
                    if (list != null && list.renderer != null && list.renderer.sharedMaterials.Length > 0)
                    {
                        foreach (Material mat in list.renderer.sharedMaterials)
                        {
                            if (mat != null && mat.shader.name.Contains("Bakery"))
                                materialSet.Add(mat);
                        }
                    }
                }

                totalUniqueMaterials = materialSet.Count;
            }

            string infoStr = "";
            infoStr += "Total replaced materials: " + numReplacedMaterials + "\n";
            infoStr += "Total renderers affected: " + rendererCount + "\n";
            infoStr += "Total original materials: " + totalUniqueOriginalMaterials + "\n";
            infoStr += "Total generated materials: " + totalUniqueMaterials;

            EditorGUILayout.HelpBox(infoStr, MessageType.Info);

            EditorGUILayout.Space();
            EditorGUILayout.Space();
            
            EditorGUILayout.LabelField("Shader Replacement", EditorStyles.boldLabel);

            if (GUILayout.Button("Standard->Bakery " + (adapter.LightmapMode == 3 ? "SH" : "RNM")))
            {
                adapter.ConvertStandardToBakeryStandard(false);
            }

            if (GUILayout.Button("Standard->Bakery " + (adapter.LightmapMode == 3 ? "SH" : "RNM") + " w/ Lightmap Specular"))
            {
                adapter.ConvertStandardToBakeryStandard(true);
            }
            
            if (adapter.replaceTransparentStandard = GUILayout.Toggle(adapter.replaceTransparentStandard, "  Replace Standard Transparent with Bakery Transparent"))
            {
                EditorGUILayout.HelpBox("This may cause your transparent objects to flicker in game! Bakery transparency will not be sorted properly in VRChat.", MessageType.Warning);
            }

            EditorGUILayout.Space();

            if (GUILayout.Button("Bakery->Standard"))
            {
                adapter.ConvertBakeryStandardToStandard();
            }

            // Horizontal spacing line
            //EditorGUILayout.LabelField("", GUI.skin.horizontalSlider);

            EditorGUILayout.Space();

            EditorGUILayout.LabelField("Debugging", EditorStyles.boldLabel);
            // Handling for loading backup reversion files in case the user accidentally deletes the replacer and loses its reversion data
            // This is mostly a last resort functionality that should only be used if the user really breaks something badly. 
            // I'm not entirely sure what conditions would need to be met to create this kind of situation at this point. This was originally made when I didn't have a decent way of detecting OnDestroy() in editor.
            if (GUILayout.Button("Load Material Reversion Profile"))
            {
                string loadPath = EditorUtility.OpenFilePanel("Open Revision File", Application.dataPath + "/" + VRCBakeryAdapter.revertProfilePath, "rev");

                int option = 0;

                if (adapter.OriginalRendererMaterials != null && adapter.OriginalRendererMaterials.Length > 0)
                {
                    option = EditorUtility.DisplayDialogComplex("Revert Materials Pending",
                                                                "This adapter still contains un-reverted materials. Do you want to revert materials before loading the new reversion profile?",
                                                                "Revert",
                                                                "Don't Revert",
                                                                "Cancel");
                }

                if (option == 0)
                {
                    adapter.RevertMaterials();
                    adapter.LoadReversionProfile(loadPath);
                }
                else if (option == 1)
                {
                    adapter.LoadReversionProfile(loadPath);
                }
            }

            EditorGUI.BeginDisabledGroup(replacerMaterialsInvalid);
            if (GUILayout.Button("Print Affected Renderers"))
            {
                adapter.OnPrintMaterials();
            }
            EditorGUI.EndDisabledGroup();
        }

        EditorGUI.EndDisabledGroup(); // Play mode disabled group
    }
}
#endif
