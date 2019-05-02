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
using System.Text.RegularExpressions;

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
    public int LightmapMode = -1; // Default to Auto which is -1
    public ReplacementScope replacementScope = ReplacementScope.Scene;
    public bool includeInactiveObjects = false;
    public string currentRevertPath = "";
    public bool compileKeywords = true;

    // Utils vars
    public bool showUtilsPane = false;
    public bool replaceTransparentStandard = false;
    
    private Dictionary<string, Shader> shaderHashToCompiledShaderMap = new Dictionary<string, Shader>();
    private Material currentWorkingMaterial = null;

    [Serializable]
    struct KeyShaderPair
    {
        public string key;
        public Shader shader;
    }
    [SerializeField]
    private List<KeyShaderPair> serializedKeyShaderPairs = new List<KeyShaderPair>();

    void OnDestroy()
    {
        if (!EditorApplication.isPlayingOrWillChangePlaymode && !EditorApplication.isPlaying)
        {
            // Don't cleanup the shaders when destroyed because OnDestroy will be called on application exit. I attempted to use the OnApplicationQuit event to detect this, but it does not seem to function.
            // So here we are; for now you will get shaders lying around if you delete the component without reverting the materials...
            RevertMaterials(false);
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

    public void RevertMaterials(bool cleanupFiles = true)
    {
        if (OriginalRendererMaterials != null)
        {
            HashSet<Material> materialsToDelete = new HashSet<Material>();

            for (int i = 0; i < OriginalRendererMaterials.Length; i++)
            {
                RendererMaterialList list = OriginalRendererMaterials[i];

                if (list != null && list.renderer != null && list.materials != null)
                {
                    foreach (Material oldMat in list.renderer.sharedMaterials)
                    {
                        materialsToDelete.Add(oldMat);
                    }

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

            // Cleanup generated shaders
            if (cleanupFiles)
            {
                for (int i = 0; i < serializedKeyShaderPairs.Count; i++)
                {
                    EditorUtility.DisplayProgressBar("Reverting Materials", "Cleaning up generated shaders", (i / (float)serializedKeyShaderPairs.Count) * 0.5f);

                    var hashShader = serializedKeyShaderPairs[i];
                    AssetDatabase.DeleteAsset(AssetDatabase.GetAssetPath(hashShader.shader));
                }

                int matCounter = 0;
                foreach (var oldMat in materialsToDelete)
                {
                    EditorUtility.DisplayProgressBar("Reverting Materials", "Cleaning up generated materials", (matCounter++ / (float)materialsToDelete.Count) * 0.5f + 0.5f);
                    AssetDatabase.DeleteAsset(AssetDatabase.GetAssetPath(oldMat));
                }
            }

            EditorUtility.ClearProgressBar();

            shaderHashToCompiledShaderMap.Clear();
            serializedKeyShaderPairs.Clear();

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

    private string StripComments(string srcStr)
    {
        // Be lazy and do a 2 pass removal of commented sections of code, first block comments, then line comments

        int currentIdx = 0;
        string uncommentedString = "";
        int copyRunLength = 0;

        // Remove block comments
        // Copy entire runs of uncommented code at a time because appending to a string char-by-char is slow as heck
        while (currentIdx + copyRunLength < srcStr.Length)
        {
            if (srcStr[currentIdx + copyRunLength] == '/' && currentIdx + copyRunLength + 1 < srcStr.Length && srcStr[currentIdx + copyRunLength + 1] == '*')
            {
                uncommentedString += srcStr.Substring(currentIdx, copyRunLength);
                currentIdx += copyRunLength;
                copyRunLength = 0;

                // This logic for handling nested comments doesn't seem necessary since it's apparently not valid syntax but I'll keep it just in case ¯\_(ツ)_/¯
                int depth = 0;
                do
                {
                    if (srcStr[currentIdx] == '/' && srcStr[currentIdx + 1] == '*')
                        depth++;
                    else if (srcStr[currentIdx] == '*' && srcStr[currentIdx + 1] == '/')
                        depth--;

                    currentIdx++;
                } while (depth > 0 && currentIdx < srcStr.Length - 1);

                currentIdx++;
            }
            else
            {
                copyRunLength++;
            }
        }

        if (copyRunLength > 0)
        {
            uncommentedString += srcStr.Substring(currentIdx, copyRunLength);
        }

        string uncommentedString2 = "";

        // Remove line comments
        StringReader reader = new StringReader(uncommentedString);
        string line = "";
        while ((line = reader.ReadLine()) != null)
        {
            int commentStartIdx = line.IndexOf("//");
            if (commentStartIdx != -1)
                line = line.Substring(0, commentStartIdx);

            uncommentedString2 += line + "\r\n";
        }

        return uncommentedString2;
    }

    // Finds the end of the current scope beginning on the codeBlockStart '{' character
    private int FindCodeBlockEnd(string srcStr, int codeBlockStart)
    {
        int blockEnd = codeBlockStart;
        int depth = 0;

        for (int i = codeBlockStart; i < srcStr.Length; i++)
        {
            if (srcStr[i] == '{')
                depth++;
            else if (srcStr[i] == '}')
                depth--;

            if (depth <= 0)
            {
                blockEnd = i;
                break;
            }
        }

        return blockEnd;
    }

    private bool IsValidKeyword(string keyword)
    {
        bool isValidKeyword = false;
        foreach (char character in keyword)
        {
            // Cull keywords that end up being _, __, ___, etc since they are culled by Unity and not valid
            if (character != '_' && character != ' ')
            {
                isValidKeyword = true;
                break;
            }
        }

        return isValidKeyword;
    }

    private int GetWhiteSpace(string line)
    {
        int whitespaceCount = 0;
        for (int i = 0; i < line.Length; i++)
        {
            if (char.IsWhiteSpace(line[i]))
                whitespaceCount++;
            else
                break;
        }

        return whitespaceCount;
    }

    private HashSet<string> ParseAvailablePassKeywords(string shaderText)
    {
        HashSet<string> availableKeywords = new HashSet<string>();
        
        // Gather shader_feature and multi_compile's
        StringReader lineReader = new StringReader(shaderText);
        string line = "";
        while ((line = lineReader.ReadLine()) != null)
        {
            int startIdx = line.IndexOf("#pragma multi_compile ");

            if (startIdx >= 0)
            {
                startIdx = line.IndexOf("multi_compile", startIdx);
            }
            else if (startIdx == -1)
            {
                startIdx = line.IndexOf("#pragma shader_feature ");
                if (startIdx >= 0)
                    startIdx = line.IndexOf("shader_feature", startIdx);
            }
            if (startIdx == -1)
                continue;

            // not needed anymore since comments are stripped as a pre-process step
            //int commentIdx = line.IndexOf("//");
            //if (commentIdx != -1 && commentIdx < startIdx)
            //    continue;

            // Skip to beginning of keyword list
            startIdx = line.IndexOf(" ", startIdx);

            string[] keywords = line.Substring(startIdx).Split(' ');

            foreach (string keyword in keywords)
            {
                if (IsValidKeyword(keyword))
                    availableKeywords.Add(keyword.Trim().ToUpper());
            }
        }

        return availableKeywords;
    }

    private string ProcessPass(Material mat, string passCode)
    {
        string processedPass = "";

        HashSet<string> passKeywords = ParseAvailablePassKeywords(passCode);
        HashSet<string> usedKeywords = new HashSet<string>();
        foreach (string keyword in mat.shaderKeywords)
        {
            if (IsValidKeyword(keyword) && passKeywords.Contains(keyword))
                usedKeywords.Add(keyword);
        }

        StringReader reader = new StringReader(passCode);
        string line = "";
        while ((line = reader.ReadLine()) != null)
        {
            // Insert the #defines for this variant
            if (line.Contains("CGPROGRAM") || line.Contains("HLSLPROGRAM"))
            {
                processedPass += line + "\r\n"; // Add the CGPROGRAM line

                int whitespaceCount = GetWhiteSpace(line);

                foreach (string keyword in usedKeywords)
                {
                    string defineStr = "#define " + keyword + " 1";
                    defineStr = defineStr.PadLeft(whitespaceCount + defineStr.Length);

                    processedPass += defineStr + "\r\n";
                }
            }
            else if (line.Contains("#pragma shader_feature ") || line.Contains("#pragma multi_compile "))
            {
#if false
                string outputStr = "// Removed shader_feature or multi_compile\r\n";
                int whitespace = GetWhiteSpace(line);
                outputStr = outputStr.PadLeft(whitespace + outputStr.Length);

                processedPass += outputStr;
#endif
            }
            else
            {
                processedPass += line + "\r\n";
            }
        }

        return "Pass " + processedPass;
    }

    private Shader GenerateShaderVariation(Material mat)
    {
        // We only care about Bakery shaders
        if (!mat.shader.name.Contains("Bakery"))
            return mat.shader;

        Shader originalShader = mat.shader;
        string shaderPath = AssetDatabase.GetAssetPath(originalShader);
        string shaderDirectory = Path.GetDirectoryName(Path.GetFullPath(shaderPath));

        string[] keywords = mat.shaderKeywords;
        Array.Sort(keywords, (x, y) => string.Compare(x, y));

        string hashStr = shaderPath;
        foreach (string keyword in keywords)
            hashStr += keyword;

        string shaderHash = ComputeMD5(hashStr);
        Shader compiledShader = null;
        if (shaderHashToCompiledShaderMap.TryGetValue(shaderHash, out compiledShader))
        {
            return compiledShader;
        }

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
            Debug.LogError("Failed to open Bakery shader to compile!");
            return mat.shader;
        }
        
        shaderText = StripComments(shaderText);

        string finalShaderCode = "";

        // Search for Passes to replace 
        int passIdx = 0;
        int lastPassIdx = 0;

        // Run through all the passes that need #define's inserted
        while (true)
        {
            lastPassIdx = passIdx;
            passIdx = shaderText.IndexOf("Pass", passIdx, StringComparison.CurrentCultureIgnoreCase);

            if (passIdx == -1)
                break;

            // Splice in all sections in between passes
            finalShaderCode += shaderText.Substring(lastPassIdx, passIdx - lastPassIdx);

            int passStartIdx = passIdx + 4;
            while (passStartIdx < shaderText.Length)
            {
                char currentChar = shaderText[passStartIdx];
                if (currentChar == '{')
                    break;
                else if (currentChar != ' ' && currentChar != '\n' && currentChar != '\r' && currentChar != '\t')
                {
                    passStartIdx = -1;
                    break;
                }

                passStartIdx++;
            }

            // This isn't the pass we're looking for... It's some other piece of code that has the word 'pass' in it.
            if (passStartIdx == -1)
            {
                finalShaderCode += shaderText.Substring(passIdx, 4);
                passIdx += 4;
                continue;
            }

            int passEndIdx = FindCodeBlockEnd(shaderText, passStartIdx) + 1;

            finalShaderCode += ProcessPass(mat, shaderText.Substring(passStartIdx, passEndIdx - passStartIdx));

            passIdx = passEndIdx;
        }

        // Patch in the last bit of code after the last pass
        if (lastPassIdx < shaderText.Length)
        {
            finalShaderCode += shaderText.Substring(lastPassIdx);
        }

        // Give the shader a unique name under Hidden/<shader hash>
        // In hindsight I probably should've just bit the bullet and did more of this using Regex
        Regex shaderNameRegex = new Regex(@"Shader\s*""\w.+""\s*{", RegexOptions.IgnoreCase);
        finalShaderCode = shaderNameRegex.Replace(finalShaderCode, "Shader \"Hidden/" + shaderHash + "\"\r\n{");

        string savePath = Path.Combine(shaderDirectory, "generated_" + shaderHash + ".shader");
        string savePathProject = Path.Combine(Path.GetDirectoryName(shaderPath), "generated_" + shaderHash + ".shader");

        try
        {
            using (StreamWriter shaderWriter = new StreamWriter(savePath))
            {
                shaderWriter.Write(finalShaderCode);
            }
        }
        catch (IOException)
        {
            Debug.LogError("Failed to write shader for variation " + mat.shader.name + " to " + savePath);
            return mat.shader;
        }

        AssetDatabase.ImportAsset(savePathProject);
        Shader generatedShader = AssetDatabase.LoadAssetAtPath<Shader>(savePathProject);

        // Add to cache so we don't regenerate again
        shaderHashToCompiledShaderMap.Add(shaderHash, generatedShader);

        return generatedShader;
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
                int propertyLightmapMode = (int)propertyBlock.GetFloat("bakeryLightmapMode");

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

                                // Add a reference to the object outside the stack to hopefully prevent a very rare bug where the material gets destroyed after a shader variation is generated, but before the asset is created.
                                currentWorkingMaterial = newMaterial;

                                newMaterial.SetTexture("_RNM0", RNM0);
                                newMaterial.SetTexture("_RNM1", RNM1);
                                newMaterial.SetTexture("_RNM2", RNM2);
                                newMaterial.SetInt("bakeryLightmapMode", LightmapMode == -1 ? propertyLightmapMode : LightmapMode); // If -1 just get the lightmap mode from the property block

                                if (compileKeywords)
                                {
                                    newMaterial.shader = GenerateShaderVariation(newMaterial);
                                    // There's no real point to this right now since Unity will detect that this object is referencing the old materials and include them in the assetbundle regardless.
                                    // The only half decent solution here is removing the keywords from the old materials temporarily as well which is pretty destructive and it's more work than I feel like at the moment to get it working and robust.
                                    // Having missing keywords will also mess with the bakes since cutout and such is detected using keywords, even though people technically shouldn't be doing bakes with materials replaced we'll make an attempt to keep the bakes working.
                                    //newMaterial.shaderKeywords = new string[0]; 
                                }

                                AssetDatabase.CreateAsset(newMaterial, savePath + "generatedbakery_" + materialFileName + "_" + matTexHash + ".mat");

                                generatedMaterialList.Add(matTexHash, newMaterial);
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
            }
        }

        AssetDatabase.SaveAssets();
        EditorSceneManager.MarkSceneDirty(gameObject.scene);

        // Backup revert settings
        SaveReversionProfile();

        // Be extra sure the materials are saved and such before the upload comes around.
        AssetDatabase.SaveAssets();
        EditorSceneManager.MarkSceneDirty(gameObject.scene);

        AssetDatabase.Refresh(ImportAssetOptions.ForceUpdate | ImportAssetOptions.ForceSynchronousImport);
        AssetDatabase.SaveAssets();

        // Flatten out the dictionary to a list so that Unity can serialize it...
        serializedKeyShaderPairs.Clear();
        foreach (var keyshader in shaderHashToCompiledShaderMap)
        {
            serializedKeyShaderPairs.Add(new KeyShaderPair() { key = keyshader.Key, shader = keyshader.Value });
        }

        shaderHashToCompiledShaderMap.Clear();
        currentWorkingMaterial = null;

        EditorUtility.ClearProgressBar();

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

    public void ConvertStandardToBakeryStandard(bool useLightmapSpecular, int lightmapModeReplacement)
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
                    if (lightmapModeReplacement == 3)
                    {
                        currentMaterial.SetInt("_BAKERY_SH", 1);
                        currentMaterial.EnableKeyword("BAKERY_SH");
                        currentMaterial.EnableKeyword("BAKERY_SHNONLINEAR");
                    }
                    else if (lightmapModeReplacement == 2)
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
    private static GUIContent compileKeywordsLabel = new GUIContent("Compile Keywords", "Strips out Unity shader keywords from Bakery shaders and generates new static shaders with the proper keywords set via #defines. This lets your shaders continue to operate even when a player has run out of keywords.");
    private static GUIContent replacementEnableSpecular = new GUIContent("Lightmap Specular", "Enables the lightmap specular option on replaced materials. This can allow your materials to get specular highlights from baked lights.");

    VRCBakeryAdapter adapter = null;
    private static GUIContent[] lightmapDirectionalityModes =
    {
        new GUIContent("Auto"),
        new GUIContent("SH"),
        new GUIContent("RNM"),
    };

    private static int[] lightmapDirectionalityModeValues =
    {
        -1,
        3,
        2,
    };

    private enum ReplacementType
    {
        SH,
        RNM,
    };

    private ReplacementType replacementType = ReplacementType.SH;
    private bool useLightmapSpecular = true;

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
        adapter.compileKeywords = EditorGUILayout.Toggle(compileKeywordsLabel, adapter.compileKeywords);

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
                            if (mat != null && (mat.shader.name.Contains("Bakery") || mat.shader.name.Contains("Hidden")))
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

            replacementType = (ReplacementType)EditorGUILayout.EnumPopup("Lightmap Mode", replacementType);

            int replacementTypeInt = 3;
            if (replacementType == ReplacementType.RNM)
                replacementTypeInt = 2;

            useLightmapSpecular = EditorGUILayout.Toggle(replacementEnableSpecular, useLightmapSpecular);

            if (adapter.replaceTransparentStandard = EditorGUILayout.Toggle("Replace Transparent Materials", adapter.replaceTransparentStandard))
            {
                EditorGUILayout.HelpBox("This may cause your transparent objects to flicker in game! Bakery transparency will not be sorted properly in VRChat.", MessageType.Warning);
            }

            if (GUILayout.Button("Standard->Bakery " + (replacementTypeInt == 3 ? "SH" : "RNM") + (useLightmapSpecular ? " /w Lightmap Specular" : "")))
            {
                adapter.ConvertStandardToBakeryStandard(useLightmapSpecular, replacementTypeInt);
            }

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
