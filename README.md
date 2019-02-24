# VRC Bakery Adapter
A script that handles converting Bakery RNM and SH directional lightmap bindings into a format that VRChat can process without the Bakery scripts being whitelisted

This script is only necessary if you are using the RNM or SH directional light modes on Bakery.

If the Bakery scripts get whitelisted, this won’t be needed and the directional lightmaps will just work in game. Please vote on the Canny for this if you have a moment: https://vrchat.canny.io/feature-requests/p/whitelist-bakery-gpu-lightmapper


#### Examples
##### Using dominant directional mode
<img src="https://i.imgur.com/oU5wqEz.jpg" width="70%" height="70%">

##### Using the SH directional mode that this script enables in VRC
<img src="https://i.imgur.com/a0nI8UH.jpg" width="70%" height="70%">


#### Why Bakery breaks in VRC and what the script does
Bakery uses a script to bind material [property blocks](https://docs.unity3d.com/ScriptReference/MaterialPropertyBlock.html) per renderer at runtime. These property blocks contain references to the directional lightmaps that Bakery uses for RNM and SH modes, the property blocks also specify the lightmap mode that the shaders use. Since these scripts aren't currently whitelisted, they cannot set these property blocks in game.

TCL originally demonstrated that adding properties to the materials for these lightmaps and then setting them manually to the Bakery lightmaps per material allowed the lightmaps to function properly in VRChat. This script automates that process and creates unique materials for each combination of material and directional lightmap binding that is used. This is necessary because the mapping between materials and property blocks on their renders is not always 1-to-1.

This also handles automatically patching the Bakery shaders to add the properties they need to work with VRC. This patch does not interfere with the usual operation of Bakery.

#### The component on an object
<img src="https://i.imgur.com/5XWPgpY.png" width="50%" height="50%">

#### Instructions
1. Make sure Bakery is imported into your project and you have baked lightmaps using the RNM or SH directional lightmap mode.
2. Download and install the latest release off the [releases](https://github.com/Merlin-san/VRC-Bakery-Adapter/releases) page. 
3. Make a new object in the scene that will handle managing the bakery materials.
4. Add a VRC Bakery Adapter component to the object.
5. Select a replacement scope for the object. The default *Scene* replacement scope should be enough for almost all situations, but if you have some odd case where you want to have different settings for different groups of objects you may use the *Children* replacement scope to only replace Bakery materials belonging to renderers on the children of the adapter object.
6. Choose a directional mode. The default SH directional mode is probably what you would want in most cases since it will generally look better, but switch this to RNM if you use Bakery's RNM directional mode.
7. In order to take advantage of SH or RNM lightmaps you need to change any lightmapped assets from using Standard to Bakery's version of Standard. There is a utility under the *Utilities* dropdown to do this automatically. Click **Standard->Bakery SH/RNM** to replace the shaders on all materials in the replacement scope with their Bakery counterparts SH or RNM will automatically be determined by the lightmap directional mode you have selected. If you want lightmap-based specular highlights click the one *w/ Lightmap Specular*. This will not replace transparent Standard materials by default since Bakery transparency does not get sorted correctly by VRC.
8. Once you are ready to upload your world to VRC, click the **Run Conversion** button. This will process all the materials into a format VRC can use.
9. ***IMPORTANT*** after you have finished uploading your world, click the **Revert Materials** button on the adapter. If you do not revert them, then any changes you make to the materials will get overwritten with old settings the next time you run the conversion.

#### Recovering from a loss of material reversion data (this should never be needed)
The component should never be able to go into a state that it still has materials replaced in the scene, but has no option to revert them. If you find some way to break it like this, open an issue for it please. 

In the off chance that you do manage end up with materials in the scene stuck set to the *generatedbakery* materials and cannot revert them with the **Revert Materials** button, there is an option at the bottom of the *Utilities* dropdown **Load Material Reversion Profile**. This will open a window at a folder which, in the case of a broken adapter component, should contain a .rev file with the scene name and the adapter object name on it. Load this profile and then try clicking **Revert Materials**.
