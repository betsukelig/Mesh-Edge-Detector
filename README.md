# Mesh-Edge-Detector
Mesh Edge Detector for First-Person Parkour System Asset of Unity 3D

AddEdgesToMesh Use:

Editor menu GioG>FPPS>AddEdges
-If your First-Person Parkour System is located in a different path, change this variable in AddEdgesToMesh
static string pointPrefabPath = "Assets/First-Person Parkour System/Assets/Prefabs/Parkour Objects/Point.prefab";

-Editor menu GioG>FPPS>RemoveEdges removes all objects named "Point"    	

![](https://raw.githubusercontent.com/betsukelig/Mesh-Edge-Detector/main/edgeDetectDemo1.gif)

![](https://raw.githubusercontent.com/betsukelig/Mesh-Edge-Detector/main/edgeDetectDemo2.gif)

If you wish to do something else with the edges...

EdgesInMesh Use:

	edgeMaxAngle = 100;// aything sharper than 100 degrees is considered an edge
	EdgesInMesh edgesInMesh = new EdgesInMesh(Transform, Mesh);
	EdgesInMesh.Edge[] edges = edgesInMesh.GetEdges(edgeMaxAngle);
	EdgesInMesh.Edge[] beveledEdges = edgesInMesh.GetBeveledEdges(edgeMaxAngle, float bevelradius =0.1f);
        
returns Edge array:

 	public struct Edge
    	{
        	public Vector3 Start;
        	public Vector3 End;
        	public Vector3 Tri1Normal;
        	public Vector3 Tri2Normal;
        	public Vector3 Tri1Pos;
        	public Vector3 Tri2Pos;
    	}
