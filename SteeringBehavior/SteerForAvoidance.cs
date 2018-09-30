 #define ANNOTATE_AVOIDOBSTACLES
using UnityEngine;
using UnityEngine.Profiling;
using UnitySteer;
using UnitySteer.Helpers;
using System.Linq;
using System.Collections.Generic;
using Priority_Queue;

/// <summary>
/// Steers a vehicle to avoid stationary obstacles
/// </summary>
/// <remarks>
/// </remarks>
[RequireComponent (typeof(AvoidanceMemory))]
[AddComponentMenu("UnitySteer/Steer/... for Avoidance")]
public class SteerForAvoidance : Steering
{
	public class PotentialDirection : PriorityQueueNode
	{
		Vector3 m_vDirection;
		
	    public Vector3 Direction { 
			get { return m_vDirection; } 
			set { m_vDirection = value; } 
		}
		
        public PotentialDirection (Vector3 vDir)
        {
            m_vDirection = vDir;
        }
	}

	#region Private fields
	[SerializeField]
	float 									_estimationTime = 2;
	AvoidanceMemory							m_memory;
	HeapPriorityQueue<PotentialDirection>	m_pqPotentialDirections	= new HeapPriorityQueue<PotentialDirection>(40);
	#endregion
	
	#region Public properties
	/// <summary>
	/// How far in the future to estimate the vehicle position
	/// </summary>
	public float EstimationTime {
		get {
			return this._estimationTime;
		}
		set {
			_estimationTime = value;
		}
	}
    #endregion

	protected override void Awake ()
	{
		base.Awake();
		m_memory = GetComponent<AvoidanceMemory>();
	}    

	protected override Vector3 CalculateForce()
	{
		m_pqPotentialDirections.Clear();
		Vector3 vTarget = Vector3.zero;
		
		if ((Vehicle.Radar.Obstacles == null || !Vehicle.Radar.Obstacles.Any()) && m_memory.IsMemoryEmpty())
		{            
            return vTarget;
		}

		bool				bIntersect		= false;
		Intersection		closestInter;
		
		Profiler.BeginSample("Accumulate obstacles");
		bIntersect = DoesFutureDirectionIntersect(Vehicle.DesiredVelocity.normalized, out closestInter);
		Profiler.EndSample();

        if (bIntersect)
        {
            m_memory.AddDetectableObject(closestInter.Obstacle);
            vTarget = FindAvoidanceVector(closestInter);            
        }        

        m_memory.CleanupMemory();
	
		return vTarget;
	}

	protected bool DoesFutureDirectionIntersect (Vector3 vFutureDirection, out Intersection collisionInfo)
	{
		Vector3			vFutureMemoryPos	= Vehicle.Position + (vFutureDirection * FlyingVehicle.SSpeed.Current * m_memory.TimeToLookAhead);
		Vector3 		vFuturePos 			= Vehicle.Position + (vFutureDirection * FlyingVehicle.SSpeed.Current * _estimationTime);
		float			fClosestSqrDist		= 0;
		Intersection	closestInter		= new Intersection(Vehicle);

		//0. Check our memory for anything that needs to be checked so that we avoid looking odd.
		for (int i = 0; i < m_memory.PrevHitObjs.Count; ++i)
		{
			DetectableObject 	obj 			= m_memory.PrevHitObjs[i].m_prevHit;
			Intersection		intersection	= obj.FindNextIntersection(Vehicle.Position, vFutureMemoryPos, Vehicle.ScaledRadius);

			if (intersection.Intersect)
			{
				float fTempSqrDist	= (intersection.Point - Vehicle.Position).sqrMagnitude;
				
				if ( Mathf.Approximately(0, fClosestSqrDist) || (fTempSqrDist < fClosestSqrDist) )
				{
					closestInter	= intersection;
					fClosestSqrDist	= fTempSqrDist;
				}
			}
			else
			{
				m_memory.UpdateDetectable(i);
			}
		}

		//1. Check the radar for anything that needs to be ensured that we avoid
		for (int nObsCount = 0; nObsCount < Vehicle.Radar.Obstacles.Count; ++nObsCount)
		{
			var 			obstacle 		= Vehicle.Radar.Obstacles[nObsCount];
			if (null == obstacle || obstacle.Equals(null) || m_memory.DoesDetectableObjectExist(obstacle))
			{
				continue;
			}
			
			Intersection	intersection	= obstacle.FindNextIntersection(Vehicle.Position, vFuturePos, Vehicle.ScaledRadius);
			
			if (intersection.Intersect)
			{
				float fTempSqrDist	= (intersection.Point - Vehicle.Position).sqrMagnitude;

				if ( Mathf.Approximately(0, fClosestSqrDist) || (fTempSqrDist < fClosestSqrDist) )
				{
					closestInter	= intersection;
					fClosestSqrDist	= fTempSqrDist;
				}
			}
		}
		
		collisionInfo	= closestInter;
		#if ANNOTATE_AVOIDOBSTACLES
		Debug.DrawLine(Vehicle.Position, collisionInfo.Point, Color.red);
		Debug.DrawLine(Vehicle.Position, vFuturePos, Color.white);
		Debug.DrawLine(Vehicle.Position, vFutureMemoryPos, Color.blue);
		#endif

		return closestInter.Intersect;
	}

	protected Vector3 FindAvoidanceVector (Intersection next)
	{
		//0. Create all necessary data structures to hold potential directions and directions we don't need
		PotentialDirection 	FutureDirection			= new PotentialDirection(Vector3.zero);
		bool				bDirectionFound			= false;
		Vector3				vResultForce			= Vector3.zero;		
		Intersection		collisionInfo			= next;
		
		//1. loop until we find a direction (this does have the potential to loop forever)
		while (!bDirectionFound)
		{			
			//A. Based on the object we collided with, get the potential directions (assign heuristics in there)
			//if (vFutureDirection != Vector3.zero)
			//{
			if (DOColliderType.Sphere == collisionInfo.Obstacle.ColliderShape)
			{
				GetAvoidanceVectorsForSphere(collisionInfo);
			}
			else if (DOColliderType.Box == collisionInfo.Obstacle.ColliderShape)
			{
				GetAvoidanceVectorsForBox(collisionInfo);
			}
			else if (DOColliderType.Capsule == collisionInfo.Obstacle.ColliderShape)
			{
				GetAvoidanceVectorsForCapsule(collisionInfo);
			}
            else if (DOColliderType.Mesh == collisionInfo.Obstacle.ColliderShape)
            {
                GetAvoidanceVectorsForMesh(collisionInfo);
            }
            else if (DOColliderType.Terrain == collisionInfo.Obstacle.ColliderShape)
            {
                GetAvoidanceVectorsForTerrain(collisionInfo);
            }
			//}
			
			//B. Select direction and test
			FutureDirection		= m_pqPotentialDirections.Dequeue();
			bDirectionFound		= true;			
			if (m_pqPotentialDirections.Count <= 0)
			{
				break;
			}
		}
		
		vResultForce = FutureDirection.Direction * Vehicle.MaxForce;

		#if ANNOTATE_AVOIDOBSTACLES
		Debug.DrawLine(Vehicle.Position, Vehicle.Position + vResultForce, Color.green);
		#endif
		return vResultForce;
	}
	
	protected void GetAvoidanceVectorsForSphere (Intersection intersection)
	{
		Vector3 vObjCenter		= intersection.Obstacle.Position;
		Vector3	vInterToCenter	= vObjCenter - intersection.Point;
		Vector3 vDesired		= intersection.Point - Vehicle.Position;
			
		//Tangents to the point of intersection and the sphere.
		Vector3.OrthoNormalize(ref vInterToCenter, ref vDesired);
		Vector3	vForward		= vDesired;
		Vector3	vBack			= -vForward;
		Vector3 vLeft			= Vector3.Cross(vInterToCenter, vForward);
		Vector3	vRight			= -vLeft;			

		m_pqPotentialDirections.Enqueue(new PotentialDirection(vForward), 
										AssignHeuristicValueToDirection(vForward, vDesired));
		m_pqPotentialDirections.Enqueue(new PotentialDirection(vBack), 
		                                AssignHeuristicValueToDirection(vBack, vDesired));
		m_pqPotentialDirections.Enqueue(new PotentialDirection(vLeft), 
		                                AssignHeuristicValueToDirection(vLeft, vDesired));
		m_pqPotentialDirections.Enqueue(new PotentialDirection(vRight), 
		                                AssignHeuristicValueToDirection(vRight, vDesired));
	}
	
	protected void GetAvoidanceVectorsForBox (Intersection intersection)
	{
		//0. Convert Intersection Point
		Vector3 vLocalInter		= intersection.Point;
		Bounds	ObsBounds		= intersection.Obstacle.MeshBounds;
		Vector3 vDesired		= intersection.Point - Vehicle.Position;
		vLocalInter				= intersection.Obstacle.Transform.InverseTransformPoint(vLocalInter);
		vLocalInter.x			= Mathf.Clamp (vLocalInter.x, ObsBounds.min.x, ObsBounds.max.x);
		vLocalInter.y			= Mathf.Clamp (vLocalInter.y, ObsBounds.min.y, ObsBounds.max.y);
		vLocalInter.z			= Mathf.Clamp (vLocalInter.z, ObsBounds.min.z, ObsBounds.max.z);
		
		//1. Find Closest Side
		Box3DSide boxSide		= intersection.Obstacle.FindIntersectedSide(vLocalInter);
		
		//2. Get the Normal
		Vector3 vTempFor		= (boxSide.vUR - boxSide.vUL).normalized;
		Vector3 vTempRight		= (boxSide.vLL - boxSide.vUL).normalized;
		Vector3 vNormal			= Vector3.Cross (vTempFor, vTempRight);		

		//3. Get the closest points of intersection
		Vector3	vInter1			= Mathfx.ClosestPtToPointOnSegment(vLocalInter, boxSide.vUL, boxSide.vUR);
		Vector3	vInter2			= Mathfx.ClosestPtToPointOnSegment(vLocalInter, boxSide.vUL, boxSide.vLL);
		Vector3	vInter3			= Mathfx.ClosestPtToPointOnSegment(vLocalInter, boxSide.vLL, boxSide.vLR);
		Vector3	vInter4			= Mathfx.ClosestPtToPointOnSegment(vLocalInter, boxSide.vUR, boxSide.vLR);
		
		//4. Do we run parallel to the shape or try to move around it
		Vector3 vLocalVehPos	= intersection.Obstacle.Transform.InverseTransformPoint(Vehicle.Position);
		Vector3 vLocalDir1		= DecideDirection (vInter1, vLocalInter, vNormal, vLocalVehPos);
		Vector3 vLocalDir2		= DecideDirection (vInter2, vLocalInter, vNormal, vLocalVehPos);
		Vector3 vLocalDir3		= DecideDirection (vInter3, vLocalInter, vNormal, vLocalVehPos);
		Vector3 vLocalDir4		= DecideDirection (vInter4, vLocalInter, vNormal, vLocalVehPos);
		
		//5. Commit to the priority queue
		vLocalDir1				= intersection.Obstacle.Transform.TransformDirection(vLocalDir1);
		vLocalDir2				= intersection.Obstacle.Transform.TransformDirection(vLocalDir2);
		vLocalDir3				= intersection.Obstacle.Transform.TransformDirection(vLocalDir3);
		vLocalDir4				= intersection.Obstacle.Transform.TransformDirection(vLocalDir4);
		
		m_pqPotentialDirections.Enqueue(new PotentialDirection(vLocalDir1), 
		                                AssignHeuristicValueToDirection(vLocalDir1, vDesired));		
		m_pqPotentialDirections.Enqueue(new PotentialDirection(vLocalDir2), 
		                                AssignHeuristicValueToDirection(vLocalDir2, vDesired));		
		m_pqPotentialDirections.Enqueue(new PotentialDirection(vLocalDir3), 
		                                AssignHeuristicValueToDirection(vLocalDir3, vDesired));		
		m_pqPotentialDirections.Enqueue(new PotentialDirection(vLocalDir4), 
		                                AssignHeuristicValueToDirection(vLocalDir4, vDesired));		
	}
	
	protected Vector3 SanityCheckDirectionAgainstNormal (Vector3 vNormal, Vector3 vLocalInter, Intersection inter)
	{
		Vector3 vResultDir;
		Vector3 vLocalVehiclePt	= inter.Obstacle.Transform.InverseTransformPoint(Vehicle.Position);
		Vector3 vCollisionDir	= vLocalInter - vLocalVehiclePt;
		float	fAngle			= Vector3.Dot (vCollisionDir, vNormal);
		
		if (Mathf.Approximately(fAngle, 1.0f) || Mathf.Approximately(fAngle, -1.0f))
		{
			Vector3 vLocalVehicleUp 	= inter.Obstacle.Transform.InverseTransformPoint (Vehicle.Transform.up);
			vCollisionDir				= (vLocalVehicleUp - vLocalVehiclePt).normalized;
			Vector3.OrthoNormalize(ref vNormal, ref vCollisionDir);
		}
		else
		{
			Vector3.OrthoNormalize(ref vNormal, ref vCollisionDir);
		}
		
		vResultDir	= vCollisionDir;
		return vResultDir;
	}
	
	protected Vector3 DecideDirection (Vector3 vSideInter, Vector3 vLocalInter, Vector3 vNormal, Vector3 vLocalVehiclePos)
	{
		Vector3 vResultDir			= Vector3.zero;
		float 	fSqDistInter		= (vSideInter - vLocalInter).sqrMagnitude;
		float 	fSqDistVehToInter	= (vLocalVehiclePos - vLocalInter).sqrMagnitude;
		Vector3	vSideDir			= (vSideInter - vLocalInter).normalized;
		
		if (fSqDistVehToInter < fSqDistInter)
		{
			//vResultDir	= vSideDir;
			vResultDir	= vSideInter + (vNormal * Vehicle.ScaledRadius * 2);
		}
		else
		{
			vResultDir	= vSideInter + (vSideDir * Vehicle.ScaledRadius) + (vNormal * Vehicle.ScaledRadius)
							+ (vSideDir * Vehicle.MaxSpeed) + (vNormal * Vehicle.MaxSpeed);
			vResultDir.Normalize();
		}
		
		return vResultDir;
	}
	
    protected void GetAvoidanceVectorsForTerrain (Intersection intersection)
    {
        Vector3 vDesired = (intersection.Point - transform.position).normalized;
        Vector3 vReflectedDir = Vector3.Reflect(vDesired, intersection.Normal);
        m_pqPotentialDirections.Enqueue(new PotentialDirection(vReflectedDir),
                                           AssignHeuristicValueToDirection(vReflectedDir, vDesired, intersection));
    }

    protected void GetAvoidanceVectorsForMesh (Intersection intersection)
    {
        Vector3 vDesired = (intersection.Point - transform.position).normalized;
        Vector3 vReflectedDir = Vector3.Reflect(vDesired, intersection.Normal);
        m_pqPotentialDirections.Enqueue(new PotentialDirection(vReflectedDir),
                                           AssignHeuristicValueToDirection(vReflectedDir, vDesired, intersection));
    }

	protected void GetAvoidanceVectorsForCapsule (Intersection intersection)
	{
		//0. Convert Intersection Point
		Vector3	vLocalInter			= intersection.Obstacle.Transform.InverseTransformPoint(intersection.Point);
		Vector3 vLocalVehiclePos	= intersection.Obstacle.Transform.InverseTransformPoint(Vehicle.Position);
		
		if (CapsuleDirection.XAxis == intersection.Obstacle.Direction)
		{
			// 0. Horizontal
			float 	fHalfCapsuleHeight	= (intersection.Obstacle.ScaledHeight / 2.0f);
			float 	fTempX				= Mathf.Clamp(vLocalInter.x, -fHalfCapsuleHeight, fHalfCapsuleHeight);
			Vector3 vTempCenter			= new Vector3 (fTempX, 0f, 0f);

			Vector3	vInterToCenter		= (vTempCenter - vLocalInter).normalized;
			Vector3 vDesired			= (vLocalInter - vLocalVehiclePos).normalized;
			Vector3 vDesiredActual      = intersection.Obstacle.Transform.TransformDirection(vDesired);
			
			//Tangents to the point of intersection and the sphere.
			Vector3.OrthoNormalize(ref vInterToCenter, ref vDesired);
			Vector3 vFlatInterToCenter	= new Vector3 (0f, vInterToCenter.y, vInterToCenter.z).normalized;
			Vector3 vFlatDesired		= new Vector3 (0f, vDesired.y, vDesired.z).normalized;
			Vector3 vWorldInterToCenter	= intersection.Obstacle.Transform.TransformDirection(vInterToCenter);
			Vector3	vForward			= intersection.Obstacle.Transform.TransformDirection(vDesired).normalized;
			Vector3	vBack				= -vForward;
			Vector3 vLeft				= Vector3.Cross(vWorldInterToCenter, vForward).normalized;
			Vector3	vRight				= -vLeft;			
			
			m_pqPotentialDirections.Enqueue(new PotentialDirection(vForward), 
			                                AssignHeuristicValueToDirection(vForward, vDesiredActual, intersection));
			m_pqPotentialDirections.Enqueue(new PotentialDirection(vBack), 
			                                AssignHeuristicValueToDirection(vBack, vDesiredActual, intersection));
			m_pqPotentialDirections.Enqueue(new PotentialDirection(vLeft), 
			                                AssignHeuristicValueToDirection(vLeft, vDesiredActual, intersection));
			m_pqPotentialDirections.Enqueue(new PotentialDirection(vRight), 
			                                AssignHeuristicValueToDirection(vRight, vDesiredActual, intersection));	
			
			vFlatInterToCenter		= intersection.Obstacle.Transform.TransformDirection(vFlatInterToCenter);
			vForward				= intersection.Obstacle.Transform.TransformDirection(vFlatDesired).normalized;
			vBack					= -vForward;
			vLeft					= Vector3.Cross(vFlatInterToCenter, vForward).normalized;
			vRight					= -vLeft;			
			
			m_pqPotentialDirections.Enqueue(new PotentialDirection(vForward), 
			                                AssignHeuristicValueToDirection(vForward, vDesiredActual, intersection));
			m_pqPotentialDirections.Enqueue(new PotentialDirection(vBack), 
			                                AssignHeuristicValueToDirection(vBack, vDesiredActual, intersection));
			m_pqPotentialDirections.Enqueue(new PotentialDirection(vLeft), 
			                                AssignHeuristicValueToDirection(vLeft, vDesiredActual, intersection));
			m_pqPotentialDirections.Enqueue(new PotentialDirection(vRight), 
			                                AssignHeuristicValueToDirection(vRight, vDesiredActual, intersection));	
			
		}
		else if (CapsuleDirection.YAxis == intersection.Obstacle.Direction)
		{
			// 0. Horizontal
			float 	fHalfCapsuleHeight	= (intersection.Obstacle.ScaledHeight / 2.0f);
			float 	fTempY				= Mathf.Clamp(vLocalInter.y, -fHalfCapsuleHeight, fHalfCapsuleHeight);
			Vector3 vTempCenter			= new Vector3 (0f, fTempY, 0f);

			Vector3	vInterToCenter		= (vTempCenter - vLocalInter).normalized;
			Vector3 vDesired			= (vLocalInter - vLocalVehiclePos).normalized;
			Vector3 vDesiredActual      = intersection.Obstacle.Transform.TransformDirection(vDesired);
			
			//Tangents to the point of intersection and the sphere.
			Vector3.OrthoNormalize(ref vInterToCenter, ref vDesired);
			Vector3 vFlatInterToCenter	= new Vector3 (vInterToCenter.x, 0f, vInterToCenter.z).normalized;
			Vector3 vFlatDesired		= new Vector3 (vDesired.x, 0f, vDesired.z).normalized;
			Vector3 vWorldInterToCenter	= intersection.Obstacle.Transform.TransformDirection(vInterToCenter);
			Vector3	vForward			= intersection.Obstacle.Transform.TransformDirection(vDesired).normalized;
			Vector3	vBack				= -vForward;
			Vector3 vLeft				= Vector3.Cross(vWorldInterToCenter, vForward).normalized;
			Vector3	vRight				= -vLeft;			
			
			m_pqPotentialDirections.Enqueue(new PotentialDirection(vForward), 
			                                AssignHeuristicValueToDirection(vForward, vDesiredActual, intersection));
			m_pqPotentialDirections.Enqueue(new PotentialDirection(vBack), 
			                                AssignHeuristicValueToDirection(vBack, vDesiredActual, intersection));
			m_pqPotentialDirections.Enqueue(new PotentialDirection(vLeft), 
			                                AssignHeuristicValueToDirection(vLeft, vDesiredActual, intersection));
			m_pqPotentialDirections.Enqueue(new PotentialDirection(vRight), 
			                                AssignHeuristicValueToDirection(vRight, vDesiredActual, intersection));	

			vFlatInterToCenter		= intersection.Obstacle.Transform.TransformDirection(vFlatInterToCenter);
			vForward				= intersection.Obstacle.Transform.TransformDirection(vFlatDesired).normalized;
			vBack					= -vForward;
			vLeft					= Vector3.Cross(vFlatInterToCenter, vForward).normalized;
			vRight					= -vLeft;			

			m_pqPotentialDirections.Enqueue(new PotentialDirection(vForward), 
			                                AssignHeuristicValueToDirection(vForward, vDesiredActual, intersection));
			m_pqPotentialDirections.Enqueue(new PotentialDirection(vBack), 
			                                AssignHeuristicValueToDirection(vBack, vDesiredActual, intersection));
			m_pqPotentialDirections.Enqueue(new PotentialDirection(vLeft), 
			                                AssignHeuristicValueToDirection(vLeft, vDesiredActual, intersection));
			m_pqPotentialDirections.Enqueue(new PotentialDirection(vRight), 
			                                AssignHeuristicValueToDirection(vRight, vDesiredActual, intersection));	
		}
		else if (CapsuleDirection.ZAxis == intersection.Obstacle.Direction)
		{
			// 0. Horizontal
			
			// 1. Vertical			
		}
	}
	
	private double AssignHeuristicValueToDirection (Vector3 vSuggestedDir, Vector3 vDesired)
	{
		double	dTotal				= 0;
		Vector3 vSuggested			= vSuggestedDir.normalized;
		//0. Add value based on how similar the Suggested is to the Desired		
		
		//1. Add value based on how similar the Suggested is to the Current
		float fCloseToCurrent		= -Vector3.Dot(Vehicle.Transform.forward.normalized, vSuggested);

		//1. Add value based on how similar the Suggested is to the Desired
		float fCloseToDesired		= -Vector3.Dot(vDesired.normalized, vSuggested);

		//2. Add value based on how similar the Suggested is to the vector that points to the center of the obstacle
		//? - weight the individual values based on distance from the actor?
		float 	fCloseToConflicted	= 0;
		float 	fCount				= 0;

		for (int i = 0; i < m_memory.PrevHitObjs.Count; ++i)
		{
			Vector3 vDirToObs	 = (m_memory.PrevHitObjs[i].m_prevHit.Position - Vehicle.Position).normalized;
			fCloseToConflicted	 += Vector3.Dot (vSuggested, vDirToObs);
			++fCount;
		}

		for (int nObsCount = 0; nObsCount < Vehicle.Radar.Obstacles.Count; ++nObsCount)
		{
			var obstacle	= Vehicle.Radar.Obstacles[nObsCount];
			if (null == obstacle || obstacle.Equals (null) || m_memory.DoesDetectableObjectExist(obstacle))
			{
				continue;
			}
			
			Vector3 vDirToObs	 = (obstacle.Position - Vehicle.Position).normalized;
			fCloseToConflicted	 += Vector3.Dot (vSuggested, vDirToObs);
			++fCount;
		}

		fCloseToConflicted			/= fCount;

		dTotal						= (double)fCloseToCurrent + (double)fCloseToConflicted + (double)fCloseToDesired;
		
		return dTotal;
	}

	private double AssignHeuristicValueToDirection (Vector3 vSuggestedDir, Vector3 vDesired, Intersection inter)
	{
		double dTotal 				= AssignHeuristicValueToDirection(vSuggestedDir, vDesired);
		dTotal						+= inter.Obstacle.SpecialHeuristicValuing(vSuggestedDir, vDesired);

		//1. Remove value based on how similar the Suggested is to the intersection direction
		float fCloseToInter			= Vector3.Dot((inter.Point - Vehicle.Position).normalized, vSuggestedDir);
		fCloseToInter				= Mathf.Clamp(fCloseToInter, 0f, 1f);
		fCloseToInter				= 3 * (Mathf.Pow (fCloseToInter, 3f));
		dTotal						+= (double)fCloseToInter;

		return dTotal;
	}

	#if ANNOTATE_AVOIDOBSTACLES
	void OnDrawGizmos()
	{
		if (Vehicle != null)
		{
			foreach (var o in Vehicle.Radar.Obstacles)
			{
				Gizmos.color = Color.red;
				Gizmos.DrawWireSphere(o.Position, o.ScaledRadius);
			}
		}
	}
	#endif
}
