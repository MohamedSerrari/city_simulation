using UnityEngine;
using UnityEngine.AI;
using System.Collections.Generic;
using Delaunay;
using Delaunay.Geo;

public class CitySimulation : MonoBehaviour
{

    public Material land;

	public GameObject road;
	public GameObject roadsHolder;

	public GameObject houseLeft;
	public GameObject houseRight;
	public GameObject housesHolder;

	public GameObject officeBuilding;
	public GameObject officesHolder;

	public NavMeshAgent agent;
	public GameObject agentsHolder;
	public NavMeshSurface surface;

	public Camera cam;

    public const int NPOINTS = 80;
    public const int NBUILDINGS = 800;
    public const int NOFFICES = 3;
    public const int NAGENTS = 10;
    public const int WIDTH = 800;
    public const int HEIGHT = 800;
	public float freqx = 0.02f, freqy = 0.018f, offsetx = 0.43f, offsety = 0.22f;

    private float scalingFactor = 20.0f;

	private List<GameObject> m_offices = new List<GameObject>();
	private List<GameObject> m_houses = new List<GameObject>();
	private List<GameObject> m_roads = new List<GameObject>();

	private List<Vector2> m_points;
	private List<LineSegment> m_edges = null;
	private List<LineSegment> m_spanningTree;
	private List<LineSegment> m_delaunayTriangulation;
	private Texture2D[] textures;
	private Texture2D tx;

	private string state = "goWork";

	private	List<Vector3> m_origins = new List<Vector3>();
	private	List<Vector3> m_destinations = new List<Vector3>();
	private	List<Vector3> agentHouses = new List<Vector3>();
	private	List<Vector3> agentOffices = new List<Vector3>();

	private float [,] createMap() 
    {
        float [,] map = new float[WIDTH, HEIGHT];
        for (int i = 0; i < WIDTH; i++)
            for (int j = 0; j < HEIGHT; j++)
                map[i, j] = Mathf.PerlinNoise(freqx * i + offsetx, freqy * j + offsety);
        return map;
    }

	void Start ()
	{
        float [,] map = createMap();
        Color[] pixels = createPixelMap(map);

		textures = Resources.LoadAll<Texture2D>("Textures");

		Debug.Log("Found textures: " + textures.Length);

        /* Create random points points */
		m_points = new List<Vector2> ();
		List<uint> colors = new List<uint> ();

		int loop = 0;
		while (loop < NPOINTS)
		{
			int x = Random.Range(1, WIDTH-2);
			int y = Random.Range(1, HEIGHT-2);
			if (MeanNeighbors(map, x, y) >= Random.Range(0.4f, 0.8f)){
				colors.Add ((uint)0);
				Vector2 vec = new Vector2(x, y); 
				m_points.Add (vec);
				loop++;
			}	
		}


		/* Generate Graphs */
		Delaunay.Voronoi v = new Delaunay.Voronoi (m_points, colors, new Rect (0, 0, WIDTH, HEIGHT));
		m_edges = v.VoronoiDiagram ();
		m_spanningTree = v.SpanningTree (KruskalType.MINIMUM);
		m_delaunayTriangulation = v.DelaunayTriangulation ();

		// Spawn roads
		Color color = Color.blue;
		for (int i = 0; i < m_edges.Count; i++) {
			LineSegment seg = m_edges [i];				
			Vector2 left = (Vector2)seg.p0;
			Vector2 right = (Vector2)seg.p1;
			SpawnRoad(left, right);
		}

		// Spawn Houses
		for (int i = 0; i < NBUILDINGS; i++)
		{
			int choice = Random.Range(0, m_edges.Count);
			LineSegment seg = m_edges[choice];

			Vector2 left = (Vector2)seg.p0;
			Vector2 right = (Vector2)seg.p1;
			SpawnHouseBuilding(left, right);
		}


		// Spawn offices
		loop = 0; 
		while (loop < NOFFICES)
		{
			int x = Random.Range(1, WIDTH-2);
			int y = Random.Range(1, HEIGHT-2);
			if (MeanNeighbors(map, x, y) >= Random.Range(0.0f, 0.8f)){
				SpawnOfficeBuilding(x, y);
				loop++;
			}	
		}

		// Apply pixels to texture
		tx = new Texture2D(WIDTH, HEIGHT);
        land.SetTexture ("_MainTex", tx);
		tx.SetPixels (pixels);
		tx.Apply ();


		// Bake the navmesh
		surface.BuildNavMesh();

		// Randomly associate each agent with a home and a work office
		SetAgentsHousesOffices();
	}

	private void SetAgentsHousesOffices(){
		
		// Give for each agent a house and an office
		for (int i = 0; i < NAGENTS; i++)
		{
			int h = Random.Range(0, m_houses.Count);
			int o = Random.Range(0, m_offices.Count);
			
			agentHouses.Add(m_houses[h].transform.position);
			agentOffices.Add(m_offices[o].transform.position);
		}
	}

	private void SpawnAgentsSetDestinations(){

		// Spawn agents in their origin places and configure their navmesh destination
		for (int i = 0; i < NAGENTS; i++)
		{
			// Instantiate road, rotate and scale
			NavMeshAgent newAgent = Instantiate(agent, m_origins[i], Quaternion.identity);
			newAgent.name = "Agent " + i;
			newAgent.transform.SetParent(agentsHolder.transform);
			
			// Set destination
			newAgent.SetDestination(m_destinations[i]);
		}
	}

    // Update is called once per frame
    void Update () 
    {
		if (state == "goWork" || state == "goHome") {

			if (state == "goWork"){
				m_origins = agentHouses; // Make origins the houses and destinations work office
				m_destinations = agentOffices;
			} else if (state == "goHome") {
				m_origins = agentOffices;
				m_destinations = agentHouses;
			} else {
				Debug.Log("Error state not found " + state);
			}

			// Instantiate agents
			SpawnAgentsSetDestinations();

			state = state == "goWork" ? "goingWork" : "goingHome";
		} 


		if (state == "goingWork" || state == "goingHome"){

			// All agents finished commuting
			int stillCommunting = agentsHolder.GetComponentsInChildren<NavMeshAgent>().Length;
			if (stillCommunting == 0)
			{
				state = state == "goingWork" ? "goHome" : "goWork";
				Debug.Log("Switched regime => state: " + state);
			}
		}
    }

	private float MeanNeighbors(float [,] map, int x, int y){
		// we dont check outside of range
		return (map[x-1, y-1] + map[x+1, y+1] + map[x+1, y] + map[x, y+1] + map[x, y])/5.0f;
	}



	private void SpawnRoad(Vector2 left, Vector2 right){

		// Position of road
		Vector3 roadPos = new Vector3((left.y - HEIGHT/2.0f) / scalingFactor, 0f, (left.x - WIDTH/2.0f) / scalingFactor);

		// Get angle and distance of road
		Vector2 alligned_vect = right - left;

		float angle = Mathf.Atan2(alligned_vect.y, alligned_vect.x) * Mathf.Rad2Deg;
		float dist = alligned_vect.magnitude;

		// Instantiate road, rotate and scale
		GameObject newRoad = Instantiate(road, roadPos, Quaternion.Euler(0, angle, 0));
		newRoad.transform.localScale = new Vector3(1f, 1f, dist / scalingFactor);
		newRoad.name = "Road " + m_roads.Count;
		newRoad.transform.SetParent(roadsHolder.transform);

		m_roads.Add(newRoad);

	}

	private void SpawnOfficeBuilding(int x, int y){
		// Spawn one office building

		// Position of road
		Vector3 pos = new Vector3((y - HEIGHT/2.0f) / scalingFactor, 0f, (x - WIDTH/2.0f) / scalingFactor);

		// Instantiate road, rotate and scale
		GameObject newOffice = Instantiate(officeBuilding, pos, Quaternion.identity);
		newOffice.transform.localScale = new Vector3(Random.Range(0.7f, 1.0f), Random.Range(1.5f, 2.0f), Random.Range(0.7f, 1.0f));
		newOffice.name = "Office " + m_offices.Count;
		newOffice.transform.SetParent(officesHolder.transform);

		m_offices.Add(newOffice);
	}

	private void SpawnHouseBuilding(Vector2 left, Vector2 right){
		// Spawn two houses next to road

		// Get angle and distance of road
		Vector2 alligned_vect = right - left;
		float angle = Mathf.Atan2(alligned_vect.y, alligned_vect.x) * Mathf.Rad2Deg;

		// Position of road
		float x_start = (left.y - HEIGHT/2.0f) / scalingFactor;
		float x_end = (right.y - HEIGHT/2.0f) / scalingFactor;

		float y_start = (left.x - WIDTH/2.0f) / scalingFactor;
		float y_end = (right.x - WIDTH/2.0f) / scalingFactor;

		// Instantiate road, rotate and scale
		float t = Random.Range(0.2f, 0.8f);
		Vector3 buildingPos = new Vector3(x_start + t*(x_end-x_start), 0, y_start + t*(y_end-y_start));


		// Chose between left or right side of the road 
		GameObject house = Random.Range(0.0f, 1.0f) > 0.5 ? houseLeft : houseRight;

		GameObject newBuilding = Instantiate(house, buildingPos, Quaternion.Euler(0, angle, 0)); // Gameobject containing multiple houses

		newBuilding.name = name;
		newBuilding.transform.SetParent(housesHolder.transform);

		// Renderer[] children = newBuilding.GetComponentsInChildren<Renderer>();

		// // give random texture
		// for (int i = 0; i < children.Length; i++)
		// {
		// 	// Select random texture
		// 	children[i].material.mainTexture = textures[Random.Range(0, textures.Length)];
		// 	children[i].transform.localScale = new Vector3(Random.Range(0.25f, 0.5f), Random.Range(0.5f, 1.5f), Random.Range(0.25f, 0.5f)); // Scale the house
		// }

		m_houses.Add(newBuilding);

	}

    /* Functions to create and draw on a pixel array */
    private Color[] createPixelMap(float[,] map)
    {
        Color[] pixels = new Color[WIDTH * HEIGHT];
        for (int i = 0; i < WIDTH; i++)
            for (int j = 0; j < HEIGHT; j++)
            {
                pixels[i * HEIGHT + j] = Color.Lerp(Color.black, Color.white, map[i, j]);
            }
        return pixels;
    }
    private void DrawPoint (Color [] pixels, Vector2 p, Color c) {
		if (p.x<WIDTH&&p.x>=0&&p.y<HEIGHT&&p.y>=0) 
		    pixels[(int)p.x*HEIGHT+(int)p.y]=c;
	}
	// Bresenham line algorithm
	private void DrawLine(Color [] pixels, Vector2 p0, Vector2 p1, Color c) {
		int x0 = (int)p0.x;
		int y0 = (int)p0.y;
		int x1 = (int)p1.x;
		int y1 = (int)p1.y;

		int dx = Mathf.Abs(x1-x0);
		int dy = Mathf.Abs(y1-y0);
		int sx = x0 < x1 ? 1 : -1;
		int sy = y0 < y1 ? 1 : -1;
		int err = dx-dy;
		while (true) {
            if (x0>=0&&x0<WIDTH&&y0>=0&&y0<HEIGHT)
    			pixels[x0*HEIGHT+y0]=c;

			if (x0 == x1 && y0 == y1) break;
			int e2 = 2*err;
			if (e2 > -dy) {
				err -= dy;
				x0 += sx;
			}
			if (e2 < dx) {
				err += dx;
				y0 += sy;
			}
		}
	}
}