# UnityPlumber
(a component to procedurally generate pipe-like meshes in Unity)

![Screenshot](https://raw.githubusercontent.com/federicocasares/unity-plumber/master/Preview.png)

## Important Information
This repository contains an example project. These files are distributed here only as part of the example scene to demonstrate the capabilities of UnityPlumber. If you are not interested in that and only want the component itself to test it out in your own project, feel free to proceed to copy only the PipeMeshGenerator.cs file and the Math3D.cs file to the Scripts directory and nothing else.

## Installation
Getting UnityPlumber running is pretty easy. You simply need to get a copy of the PipeMeshGenerator.cs and Math3D.cs files in the Scripts directory and copy them to your project. Once there, simply add the PipeMeshGenerator component to an empty GameObject, set the options and you are good to go!

## What do the options mean?
Almost all of the settings are pretty much self explanatory, but here are the details:

* points: A list of Vector3 containing all points the pipe should connect.
* pipeRadius: The radius for the cross section of the pipe.
* elbowRadius: The radius for the elbows that are automatically generated between each segment.
* pipeSegments: The number of faces in each pipe segment. Higher numbers mean smoother (rounder) looking pipes. Recommended values: 8 to 16.
* elbowSegments: The number of cross-sections to generate in each elbow where the pipe bends. Recommended values: 6 to 10.
* pipeMaterial: An Unity Material that will be applied to the MeshRenderer component in order to render the mesh.
* flatShading: Enabling this option will disable smoothing and make a flat-shaded low-poly style pipe. Enabling this will increase vertex count greatly!
* avoidStrangling: Enabling this option will cause the algorithm to try to avoid twists in elbows. This usually fixes some problems and causes new ones, so experiment to see what works best in your particular case.
* generateEndCaps: Enabling this option will automatically generate circular end caps on each end of the pipe.
* generateElbows: Enabling this option will make pipe segments be connected by toroidal arc elbows. If the option is off, only the straight segments of the tubes will be generated.
* generateOnStart: Enabling this option will make the component generate the mesh automatically when started. Disable it if you need to set the points manually in runtime first and generate the mesh later.
* makeDoubleSided: Duplicates all faces with inverted normals so your pipes will be visible not only from the outside but also from the inside. What matters is what's on the inside, after all.
* colinearThreshold: The algorithm will automatically remove any colinear points in your points list in order to improve performance and avoid ending up generating elbows that would require an infinite radius. Change this value to determine how sensitive the removal of those almost-colinear points should be. The higher the value, the higher the number of points that will end up being removed.

## Public Methods

### RenderPipe()
Generates the pipe mesh with the specified options. Automatically adds MeshFilter and MeshRenderer components to the object if they are not present.

## Math 3D Helper Class

This project makes use of the Math3D.cs file, which has been obtained from https://github.com/kristofe/UnityVolumeOculus, another project distributed under the MIT License.
