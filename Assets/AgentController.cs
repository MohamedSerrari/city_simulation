using UnityEngine;
using UnityEngine.AI;

public class AgentController : MonoBehaviour
{

    NavMeshAgent agent;

    // Start is called before the first frame update
    void Start()
    {
        agent = this.GetComponent<NavMeshAgent>();
    }

    // Update is called once per frame
    void Update()
    {
        if (agent.hasPath){
            Vector3 toTarget = agent.steeringTarget - agent.gameObject.transform.position;
            float turnAngle = Vector3.Angle(agent.gameObject.transform.forward, toTarget);
            agent.acceleration = turnAngle * agent.speed;
        }

        if (!agent.pathPending && agent.remainingDistance < 0.5){
            Debug.Log("Destroyed: " + agent.name);
            Destroy(gameObject);
            // Destroy(agent);
        }
    }
}
