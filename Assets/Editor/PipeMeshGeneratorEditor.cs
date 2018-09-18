using System.Collections;
using System.Collections.Generic;
using UnityEditor;
using UnityEngine;

[CustomEditor(typeof(PipeMeshGenerator))]
public class PipeMeshGeneratorEditor : Editor {

	public override void OnInspectorGUI() {
		DrawDefaultInspector();
		PipeMeshGenerator myTarget = (PipeMeshGenerator) target;

		if (GUILayout.Button("Generate Mesh")) {
			myTarget.RenderPipe();
		}
	}

}