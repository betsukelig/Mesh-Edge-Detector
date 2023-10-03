# Mesh-Edge-Detector
Mesh Edge Detector for [First-Person Parkour System](https://assetstore.unity.com/packages/templates/packs/first-person-parkour-system-v2-0-for-playmaker-168546) Asset of Unity 3D

AddEdgesToMesh Use:

If your First-Person Parkour System is located in a different path, change this variable in AddEdgesToMesh

static string pointPrefabPath = "Assets/First-Person Parkour System/Assets/Prefabs/Parkour Objects/Point.prefab";

-Select an Object with a MeshRenderer

-Editor menu GioG>FPPS>AddEdges   	

A mesh collider will be added to your Object as well 

![](https://raw.githubusercontent.com/betsukelig/Mesh-Edge-Detector/main/edgeDetectDemo1.gif)

-Editor menu GioG>FPPS>AddBeveledEdges  

![](https://raw.githubusercontent.com/betsukelig/Mesh-Edge-Detector/main/edgeDetectDemo2.gif)

-Editor menu GioG>FPPS>TopDownEdgesOnly  

I added this in case your world doesn't rotate and you only need top down edge points. 

![](https://raw.githubusercontent.com/betsukelig/Mesh-Edge-Detector/main/edgeDetectDemo3.gif)

-Editor menu GioG>FPPS>RemoveEdges simply removes all objects named "Point" 

If you wish to do something else with the edges...

EdgesInMesh Use:

	edgeMaxAngle = 100;// aything sharper than 100 degrees is considered an edge
	EdgesInMesh edgesInMesh = new EdgesInMesh(Transform, Mesh);
	EdgesInMesh.Edge[] edges = edgesInMesh.GetEdges(edgeMaxAngle);
	EdgesInMesh.Edge[] beveledEdges = edgesInMesh.GetBeveledEdges(edgeMaxAngle, float bevelradius =0.1f);
        
will return Edge array:

 	public struct Edge
    	{
        	public Vector3 Start;
        	public Vector3 End;
        	public Vector3 Tri1Normal;
        	public Vector3 Tri2Normal;
        	public Vector3 Tri1Pos;
        	public Vector3 Tri2Pos;
    	}
