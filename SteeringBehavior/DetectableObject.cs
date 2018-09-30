using UnityEngine;
using UnitySteer;

public enum DOColliderType 		{ None, Sphere, Box, Capsule, Mesh, Terrain };
public enum BoxSide 			{ None, MaxX, MinX, MaxY, MinY, MaxZ, MinZ };
public enum CapsuleDirection 	{ XAxis, YAxis, ZAxis, None };

public struct Intersection
{
	bool _intersect;
	float _distance;
	DetectableObject _obstacle;
	Vector3 _point;
    Vector3 _normal;
	
	public bool Intersect 
	{ 
		get { return _intersect; }
		set { _intersect = value; }
	}
	
	public float Distance 
	{ 
		get { return _distance; }
		set { _distance = value; }
	}
	
	public Vector3 Point
	{
		get { return _point; }
		set { _point = value; }
	}

    public Vector3 Normal
    {
        get { return _normal; }
        set { _normal = value; }
    }

	public DetectableObject Obstacle 
	{ 
		get { return _obstacle; } 
		set { _obstacle = value; }
	}
	
	public Intersection (DetectableObject obstacle)
	{
		_obstacle 	= obstacle;
		_intersect 	= false;
		_distance 	= float.MaxValue;
		_point		= Vector3.zero;
        _normal     = Vector3.zero;
	}
};	

public struct Box3DSide
{
	public Vector3 vUL;
	public Vector3 vLL;
	public Vector3 vUR;
	public Vector3 vLR;
	public Vector3 vCenter;
	public BoxSide bSide;
	
	public Box3DSide (Vector3 vUL1, Vector3 vLL1, Vector3 vUR1, Vector3 vLR1, Vector3 vCenter1, BoxSide side)
	{
		vUL 	= vUL1;
		vLL 	= vLL1;
		vUR		= vUR1;
		vLR 	= vLR1;
		vCenter	= vCenter1;
		bSide 	= side;
	}
}

/// <summary>
/// Parent class for objects that vehicles can aim for, be it other vehicles or
/// static objects.
/// </summary>
[AddComponentMenu("UnitySteer/Detectables/DetectableObject")]
public class DetectableObject : MonoBehaviour
{	
	float _squaredRadius;
	
	/// <summary>
	/// The vehicle's center in the transform
	/// </summary>
	[SerializeField]
	[HideInInspector]
	Vector3 _center;
	
	/// <summary>
	/// The vehicle's center in the transform, scaled to by the transform's lossyScale
	/// </summary>
	[SerializeField]
	[HideInInspector]
	Vector3 _scaledCenter;
	
	/// <summary>
	/// The vehicle's radius, scaled by the maximum of the transform's lossyScale values
	/// </summary>
	[SerializeField]
	[HideInInspector]
	float _scaledRadius = 1;

	/// <summary>
	/// The vehicle's radius.
	/// </summary>
	[SerializeField]
	[HideInInspector]
	float _radius = 1;
	
	[SerializeField]
	protected bool _drawGizmos = false;
	
	[SerializeField]
	protected DOColliderType _colliderType = DOColliderType.Sphere;	

	[SerializeField]
	private Collider			_collider;
	private Bounds	 			_bounds;
	private float				_height;
	private float				_capHeight;
	private CapsuleDirection	_direction;
	
	/// <summary>
	/// Vehicle's position
	/// </summary>
	/// <remarks>The vehicle's position is the transform's position displaced 
	/// by the vehicle center</remarks>
	public Vector3 Position {
		get {
			if (Transform == null) {
				Transform = GetComponent<Transform>();
			}
			return Transform.position + _scaledCenter;
		}
	}
	
	/// <summary>
	/// Vehicle center on the transform
	/// </summary>
	/// <remarks>
	/// This property's setter recalculates a temporary value, so it's
	/// advised you don't re-scale the vehicle's transform after it has been set
	/// </remarks>
	public Vector3 Center {
		get {
			return this._center;
		}
		set {
			_center = value;
			RecalculateScaledValues();
		}
	}
	
	/// <summary>
	/// Vehicle radius
	/// </summary>
	/// <remarks>
	/// This property's setter recalculates a temporary value, so it's
	/// advised you don't re-scale the vehicle's transform after it has been set
	/// </remarks>
	public float Radius {
		get {
			return _radius;
		}
		set {
			_radius = Mathf.Clamp(value, 0.01f, float.MaxValue);
			RecalculateScaledValues();			
		}
	}

	/// <summary>
	/// The vehicle's center in the transform, scaled to by the transform's lossyScale
	/// </summary>
	public Vector3 ScaledCenter {
		get {
			return this._scaledCenter;
		}
	}
	
	/// <summary>
	/// The vehicle's radius, scaled by the maximum of the transform's lossyScale values
	/// </summary>
	public float ScaledRadius {
		get {
			return this._scaledRadius;
		}
	}

	public float SquaredRadius {
		get {
			return this._squaredRadius;
		}
	}
	
	public DOColliderType ColliderShape 
	{
		get
		{
			return this._colliderType;
		}
	}
	
	public float Height
	{
		get
		{
			return (DOColliderType.Capsule == this._colliderType) ? this._height : this._scaledRadius;
		}
	}

	public float ScaledHeight
	{
		get
		{
			return _capHeight;
		}
	}

	public CapsuleDirection Direction
	{
		get
		{
			return this._direction;
		}
	}
	
	public Bounds MeshBounds
	{
		get { return _bounds; }
	}

    public Collider MCollider
    {
        get { return _collider; }
    }

    /// <summary>
    /// Cached transform for this behaviour
    /// </summary>
    public Transform Transform { get; private set; }		
	
	#region Methods
	protected virtual void Awake()
	{       
		Transform 				= GetComponent<Transform>();

		if (null == _collider)
		{
			_collider				= GetComponent<Collider>();
		}

        if (_collider != null)
        {
            _bounds = _collider.bounds;

            DetectDOColliderType();

            if (DOColliderType.Box == _colliderType)
            {
                BoxCollider bCollider = _collider as BoxCollider;
                Radius = bCollider.bounds.extents.magnitude;
                _direction = CapsuleDirection.None;
                _height = bCollider.bounds.extents.magnitude;
            }
            else if (DOColliderType.Sphere == _colliderType)
            {
                SphereCollider sCollider = _collider as SphereCollider;
                Radius = sCollider.radius;
                _direction = CapsuleDirection.None;
                _height = sCollider.radius;
            }
            else if (DOColliderType.Capsule == _colliderType)
            {
                CapsuleCollider cCollider = _collider as CapsuleCollider;
                Radius = cCollider.radius;
                _height = cCollider.height;
                _direction = (CapsuleDirection)cCollider.direction;
            }

            RecalculateScaledValues();

            DetectableObjectManager.Instance.Register(this);
        }
        else
        {
            enabled = false;
        }
	}

    private void OnEnable()
    {
        DetectableObjectManager.Instance.Register(this);
    }

    private void OnDisable()
    {
        if (DetectableObjectManager.Instance != null)
            DetectableObjectManager.Instance.DeRegister(this);
    }

    /// <summary>
    /// Predicts where the vehicle will be at a point in the future
    /// </summary>
    /// <param name="predictionTime">
    /// A time in seconds for the prediction <see cref="System.Single"/>. Disregarded on the base function.
    /// </param>
    /// <returns>
    /// Vehicle position<see cref="Vector3"/>
    /// </returns>
    public virtual Vector3 PredictFuturePosition(float predictionTime)
    {
        return Transform.position;
	}
	
	public virtual Box3DSide FindIntersectedSide (Vector3 vLocalInter)
	{
		Vector3 vUL 	= Vector3.zero;
		Vector3 vUR		= Vector3.zero;
		Vector3 vLL 	= Vector3.zero;
		Vector3 vLR 	= Vector3.zero;	
		Vector3	vCenter	= Vector3.zero;
		Vector3 vMax	= MeshBounds.max;
		Vector3	vMin	= MeshBounds.min;		
		BoxSide boxSide	= BoxSide.None;
		
		if (vLocalInter.y >= vMax.y)
		{
			vUL		= new Vector3 (vMin.x, vMax.y, vMax.z);
			vUR		= vMax;
			vLL		= new Vector3 (vMin.x, vMax.y, vMin.z);
			vLR		= new Vector3 (vMax.x, vMax.y, vMin.z);
			vCenter	= new Vector3 (MeshBounds.center.x, vMax.y, MeshBounds.center.z);
			boxSide = BoxSide.MaxY;
		}
		else if (vLocalInter.y <= vMin.y)
		{
			vUL		= new Vector3 (vMin.x, vMin.y, vMax.z);
			vUR		= new Vector3 (vMax.x, vMin.y, vMax.z);
			vLL		= vMin;
			vLR		= new Vector3 (vMax.x, vMin.y, vMin.z); 
			vCenter	= new Vector3 (MeshBounds.center.x, vMin.y, MeshBounds.center.z);
			boxSide = BoxSide.MinY;
		}
		else if (vLocalInter.x >= vMax.x)
		{
			vUL		= new Vector3 (vMax.x, vMax.y, vMin.z);
			vUR 	= vMax;
			vLL		= new Vector3 (vMax.x, vMin.y, vMin.z);
			vLR 	= new Vector3 (vMax.x, vMin.y, vMax.z);
			vCenter	= new Vector3 (vMax.x, MeshBounds.center.y, MeshBounds.center.z);
			boxSide = BoxSide.MaxX;
		}
		else if (vLocalInter.x <= vMin.x)
		{
			vUL 	= new Vector3 (vMin.x, vMax.y, vMax.z);
			vUR 	= new Vector3 (vMin.x, vMax.y, vMin.z);
			vLL 	= new Vector3 (vMin.x, vMin.y, vMax.z);
			vLR 	= vMin;
			vCenter	= new Vector3 (vMin.x, MeshBounds.center.y, MeshBounds.center.z);
			boxSide = BoxSide.MinX;
		}
		else if (vLocalInter.z >= vMax.z)
		{
			vUL 	= vMax;
			vUR 	= new Vector3 (vMin.x, vMax.y, vMax.z);
			vLL 	= new Vector3 (vMax.x, vMin.y, vMax.z);
			vLR 	= new Vector3 (vMin.x, vMin.y, vMax.z);
			vCenter	= new Vector3 (MeshBounds.center.x, MeshBounds.center.y, vMax.z);
			boxSide = BoxSide.MaxZ;
		}
		else if (vLocalInter.z <= vMax.z)
		{
			vUL 	= new Vector3 (vMin.x, vMax.y, vMin.z);
			vUR 	= new Vector3 (vMax.x, vMax.y, vMin.z);
			vLL 	= vMin;
			vLR 	= new Vector3 (vMax.x, vMin.y, vMin.z);
			vCenter	= new Vector3 (MeshBounds.center.x, MeshBounds.center.y, vMin.z);
			boxSide = BoxSide.MinZ;
		}		
		
		return new Box3DSide (vUL, vLL, vUR, vLR, vCenter, boxSide);
	}	
	
	public Intersection FindNextIntersection (Vector3 vVehiclePosition, Vector3 vFutureVehiclePosition, float fVehicleRad)
	{
		Intersection intersection = new Intersection(this);
		
		if (DOColliderType.Capsule == _colliderType)
		{
			intersection			= FindNextIntersectionWithCapsule(vVehiclePosition, vFutureVehiclePosition, fVehicleRad);
		}
		else if (DOColliderType.Sphere == _colliderType)
		{
			intersection			= FindNextIntersectionWithSphere (vVehiclePosition, vFutureVehiclePosition, fVehicleRad);
		}
		else if (DOColliderType.Box == _colliderType)
		{
			intersection			= FindNextIntersectionWithBox (vVehiclePosition, vFutureVehiclePosition, fVehicleRad);
		}
        else if (DOColliderType.Mesh == _colliderType)
        {
            intersection = FindNextIntersectionWithMesh(vVehiclePosition, vFutureVehiclePosition, fVehicleRad);
        }
        else if (DOColliderType.Terrain == _colliderType)
        {
            intersection = FindNextIntersectionWithTerrain(vVehiclePosition, vFutureVehiclePosition, fVehicleRad);
        }
		
		return intersection;
	}

	public void DetectDOColliderType ()
	{
		if (_collider.GetType() == typeof(SphereCollider))
		{
			_colliderType = DOColliderType.Sphere;
		}
		else if (_collider.GetType() == typeof(CapsuleCollider))
		{
			_colliderType = DOColliderType.Capsule;
		}
		else if (_collider.GetType() == typeof(BoxCollider))
		{
			_colliderType = DOColliderType.Box;
		}
        else if (_collider.GetType() == typeof(MeshCollider))
        {
            _colliderType = DOColliderType.Mesh;
        }
    }

	public double SpecialHeuristicValuing (Vector3 vSuggestedDir, Vector3 vDesired)
	{
		double dResult = 0;
	
		if (DOColliderType.Capsule == _colliderType && CapsuleDirection.YAxis == _direction)
		{
			dResult = (double)Mathf.Abs (vSuggestedDir.y * 3.0f);
		}
		else if (DOColliderType.Capsule == _colliderType && CapsuleDirection.XAxis == _direction)
		{
			Vector3 vLocal = Transform.InverseTransformDirection(vSuggestedDir);
			dResult = (double)Mathf.Abs (vLocal.y * 3.0f) + (double)Mathf.Abs (vSuggestedDir.z * 3.0f) - (double)Mathf.Abs (vSuggestedDir.y * 3.0f);
		}

		return dResult;
	}

    protected Intersection FindNextIntersectionWithMesh (Vector3 vVehiclePosition, Vector3 vFutureVehiclePosition, float fVehicleRad)
    {
        Intersection intersection = new Intersection(this);

        LayerMask mask = LayerMask.GetMask("Scenery");
        RaycastHit hit;
        bool bHit = false;
        float fRayDist = (vFutureVehiclePosition - vVehiclePosition).magnitude + fVehicleRad;
        Ray ray = new Ray(vVehiclePosition, (vFutureVehiclePosition - vVehiclePosition).normalized);

        bHit = Physics.Raycast(ray, out hit, fRayDist, mask, QueryTriggerInteraction.Collide);

        if (bHit)
        {
            intersection.Intersect = true;
            intersection.Distance = hit.distance;
            intersection.Normal = hit.normal;
            intersection.Point = hit.point;
        }

        return intersection;
    }

    protected Intersection FindNextIntersectionWithTerrain(Vector3 vVehiclePosition, Vector3 vFutureVehiclePosition, float fVehicleRad)
    {
        Intersection intersection = new Intersection(this);

        LayerMask mask = LayerMask.GetMask("Scenery");
        RaycastHit hit;
        bool bHit = false;
        float fRayDist = (vFutureVehiclePosition - vVehiclePosition).magnitude + fVehicleRad;
        Ray ray = new Ray(vVehiclePosition, (vFutureVehiclePosition - vVehiclePosition).normalized);

        bHit = Physics.Raycast(ray, out hit, fRayDist, mask, QueryTriggerInteraction.Collide);

        if (bHit)
        {
            intersection.Intersect = true;
            intersection.Distance = hit.distance;
            intersection.Normal = hit.normal;
            intersection.Point = hit.point;
        }

        return intersection;
    }

    /// <summary>
    /// Finds the next intersection with capsule.  Assumes that the vehicle collider is spherical.
    /// </summary>
    /// <returns>
    /// The next intersection with capsule.
    /// </returns>
    /// <param name='vVehiclePosition'>
    /// V vehicle postion.
    /// </param>
    /// <param name='vFutureVehiclePosition'>
    /// V future vehicle position.
    /// </param>
    protected Intersection FindNextIntersectionWithCapsule (Vector3 vVehiclePosition, Vector3 vFutureVehiclePosition, float fVehicleRad)
	{
		Intersection intersection 	= new Intersection(this);
		
		//0.  Get the needed info from the capsule
		Vector3	vStartCap				= new Vector3 ();
		Vector3 vEndCap					= new Vector3 ();
		float 	fRad					= Radius;
		GetCapsuleEndpoints	(out vStartCap, out vEndCap, false);
		fRad							+= fVehicleRad;
		
		//1. Convert the given info to the local space of the capsule
		Vector3	vLocalVehiclePos		= Transform.InverseTransformPoint(vVehiclePosition);
		Vector3	vLocalFutureVehiclePos	= Transform.InverseTransformPoint(vFutureVehiclePosition);
		
		float	fT						= 0f;
		intersection.Intersect			= SegmentCapsuleIntersection (vLocalVehiclePos, vLocalFutureVehiclePos, vStartCap, vEndCap, fRad, out fT);
		intersection.Point				= vVehiclePosition + (fT * (vFutureVehiclePosition - vVehiclePosition));

		bool	bIntersectsFurther		= (vFutureVehiclePosition - vVehiclePosition).sqrMagnitude < 
											(intersection.Point - vVehiclePosition).sqrMagnitude;
		
		if (intersection.Intersect && bIntersectsFurther)
		{
			intersection.Intersect = false;
		}

		return intersection;
	}
	
	protected bool SegmentCapsuleIntersection (Vector3 vVehiclePosition, Vector3 vFutureVehiclePosition, Vector3 vStartCap, Vector3 vEndCap, float fRad, out float fT)
	{
		fT			= 0f;
		Vector3 d 	= vEndCap - vStartCap;
		Vector3 m 	= vVehiclePosition - vStartCap;
		Vector3 n 	= vFutureVehiclePosition - vVehiclePosition;
		
		float 	md	= Vector3.Dot (m, d);
		float	nd	= Vector3.Dot (n, d);
		float	dd	= Vector3.Dot (d, d);
		
		if (md < 0.0f && md + nd < 0.0f)
		{
			return false;
		}
		if (md > dd && md + nd > dd)
		{
			return false;
		}
		
		float	nn 	= Vector3.Dot (n, n);
		float	mn 	= Vector3.Dot (m, n);
		float	a	= dd * nn - nd * nd;
		float	k 	= Vector3.Dot (m, m) - fRad * fRad;
		float 	c	= dd * k - md * md;
		
		if (Mathf.Abs(a) < Mathf.Epsilon)
		{
			if (c > 0.0f)
			{
				return false;
			}
			
			if (md < 0.0f)
			{
				fT = -mn / nn;
			}
			else if (md > dd)
			{
				fT = (nd - mn) / nn;
			}
			else
			{
				fT = 0f;
			}
			
			return true;
		}
		
		float	b		= dd * mn - nd * md;
		float 	discr	= b * b - a * c;
		
		if (discr < 0.0f)
		{
			return false;
		}
		
		fT 				= (-b - Mathf.Sqrt(discr)) / a;
		float 	fTemp	= 0f;
		Vector3	vTemp	= Vector3.zero;
		
		if (IntersectRaySphere(vVehiclePosition, vFutureVehiclePosition, vStartCap, fRad, out fTemp, out vTemp))
		{
			bool 	bWithin			= fTemp < 0.0f || fTemp > 1.0f;
			bool 	bIsCloser 		= fTemp < fT || (fT < 0.0f || fT > 1.0f);
			bool	bIsNotWithin	= (vTemp.y < vStartCap.y) && (vTemp.y > vEndCap.y);

			if (bWithin && bIsCloser && bIsNotWithin)
				fT = fTemp;

			return true;
		}
		else if (IntersectRaySphere(vVehiclePosition, vFutureVehiclePosition, vEndCap, fRad, out fTemp, out vTemp))
		{
			bool 	bWithin			= fTemp < 0.0f || fTemp > 1.0f;
			bool 	bIsCloser 		= fTemp < fT || (fT < 0.0f || fT > 1.0f);
			bool	bIsNotWithin	= (vTemp.y < vStartCap.y) && (vTemp.y > vEndCap.y);
			
			if (bWithin && bIsCloser && bIsNotWithin)
				fT = fTemp;

			return true;
		}

		if (fT < 0.0f || fT > 1.0f)
		{
			return false;
		}
		
		return true;
	}
	
	protected void GetCapsuleEndpoints (out Vector3 vStartCap, out Vector3 vEndCap, bool bScaled = true)
	{
		float fHeight = ScaledHeight;

		if (!bScaled)
			fHeight	= Height;

		vStartCap = new Vector3();
		vEndCap	= new Vector3();
		
		if (fHeight < 0f)
		{
			fHeight	= 0f;
		}

		CapsuleCollider capCollider		= _collider as CapsuleCollider; 

		if (CapsuleDirection.XAxis == Direction)
		{
			vStartCap.x	-= capCollider.center.x + (fHeight / 2f);
			vEndCap.x	+= capCollider.center.x + (fHeight / 2f);
		}
		else if (CapsuleDirection.YAxis == Direction)
		{
			vStartCap.y	-= capCollider.center.y + (fHeight / 2f);
			vEndCap.y	+= capCollider.center.y + (fHeight / 2f);
		}
		else 
		{
			vStartCap.z	-= capCollider.center.z + (fHeight / 2f);
			vEndCap.z	+= capCollider.center.z + (fHeight / 2f);
		}
	}
	
	protected Intersection FindNextIntersectionWithBox (Vector3 vVehiclePosition, Vector3 vFuturePostion, float fVehicleRad)
	{
		Intersection intersection 	= new Intersection(this);
		
		//0. Convert incoming positions into the local space
		Vector3	vLocalVehiclePos		= Transform.InverseTransformPoint(vVehiclePosition);
		Vector3 vLocalFuturePos			= Transform.InverseTransformPoint(vFuturePostion);
		float	fT						= 0f;
		
		intersection.Intersect			= MovingSphereAABBIntersection (vLocalVehiclePos, vLocalFuturePos, fVehicleRad, out fT);
		intersection.Point				= vVehiclePosition + ((vFuturePostion - vVehiclePosition) * fT);

		bool	bIntersectsFurther		= (vFuturePostion - vVehiclePosition).sqrMagnitude < 
											(intersection.Point - vVehiclePosition).sqrMagnitude;
		
		if (intersection.Intersect && bIntersectsFurther)
		{
			intersection.Intersect = false;
		}

		return intersection;
	}
	
	protected bool MovingSphereAABBIntersection (Vector3 vVehiclePosition, Vector3 vFuturePostion, float fVehicleRad, out float fT)
	{
		Vector3	vBoxMin	= Transform.InverseTransformPoint(_bounds.min);
		Vector3	vBoxMax	= Transform.InverseTransformPoint(_bounds.max);
		fT	= 0f;
		
		vBoxMin.x	-= fVehicleRad;
		vBoxMin.y	-= fVehicleRad;
		vBoxMin.z	-= fVehicleRad;
		vBoxMax.x	+= fVehicleRad;
		vBoxMax.y	+= fVehicleRad;
		vBoxMax.z	+= fVehicleRad;
		
		Vector3	vInter = Vector3.zero;
		Vector3	vDir = (vFuturePostion - vVehiclePosition);
		bool bIntersects = IntersectRayAABB (vVehiclePosition, vDir, vBoxMin, vBoxMax, out fT, out vInter);
		bool bIntersectsFurther = (vFuturePostion - vVehiclePosition).sqrMagnitude < 
											(vInter - vVehiclePosition).sqrMagnitude;

		if (!bIntersects || fT > 1f)
		{
			return false;
		}
		
		int nU = 0;
		int nV = 0;
		
		if (vInter.x < vBoxMin.x) nU |= 1;
		if (vInter.x > vBoxMax.x) nV |= 1;
		if (vInter.y < vBoxMin.y) nU |= 2;
		if (vInter.y > vBoxMax.y) nV |= 2;
		if (vInter.z < vBoxMin.z) nU |= 4;
		if (vInter.z > vBoxMax.z) nV |= 4;
		
		int nM	= nU + nV;

 		Bounds localBounds = new Bounds(Transform.InverseTransformPoint(_bounds.center), _bounds.size);

		if (nM == 7)
		{
			float fTmin	= float.MaxValue;
			
			if (SegmentCapsuleIntersection(vVehiclePosition, vFuturePostion, 
											GetBoxCorner(localBounds, nV), GetBoxCorner(localBounds, nV ^ 1), 
											fVehicleRad, out fT))
			{
				fTmin = Mathf.Min(fT, fTmin);
			}
			if (SegmentCapsuleIntersection(vVehiclePosition, vFuturePostion, 
											GetBoxCorner(localBounds, nV), GetBoxCorner(localBounds, nV ^ 2), 
											fVehicleRad, out fT))
			{
				fTmin = Mathf.Min(fT, fTmin);
			}
			if (SegmentCapsuleIntersection(vVehiclePosition, vFuturePostion, 
											GetBoxCorner(localBounds, nV), GetBoxCorner(localBounds, nV ^ 4), 
											fVehicleRad, out fT))
			{
				fTmin = Mathf.Min(fT, fTmin);
			}
			if (Mathf.Approximately(fTmin, float.MaxValue))
			{
				return false;
			}
			
			fT = fTmin;
			
			return true;
		}
		
		if ((nM & (nM - 1)) == 0)
		{
			return true;
		}
		
		return SegmentCapsuleIntersection(vVehiclePosition, vFuturePostion, 
											GetBoxCorner(localBounds, nU ^ 7), GetBoxCorner(localBounds, nV), 
											fVehicleRad, out fT);
	}
	
	protected Vector3 GetBoxCorner (Bounds bBox, int nCorner)
	{
		Vector3	vCorner = Vector3.zero;
		vCorner.x	= ((nCorner & 1) > 0) ? bBox.max.x : bBox.min.x;
		vCorner.y	= ((nCorner & 2) > 0) ? bBox.max.y : bBox.min.y;
		vCorner.z	= ((nCorner & 4) > 0) ? bBox.max.z : bBox.min.z;
		
		return vCorner;
	}
				
	protected bool IntersectRayAABB (Vector3 vVehiclePosition, Vector3 vDirection, Vector3 vBoxMin, Vector3 vBoxMax, out float fT, out Vector3 vInter)
	{	
		fT = 0f;
		float fT1 = 0f;
		float fT2 = 0f;
		vInter = Vector3.zero;
		float fTMax = float.MaxValue;

		for (int i = 0; i < 3; i++)
		{
			if (Mathf.Abs(vDirection[i]) < Mathf.Epsilon)
			{
				if ((vVehiclePosition[i] < vBoxMin[i]) || 
					(vVehiclePosition[i] > vBoxMax[i]))
				{
					return false;
				}
			}
			else
			{
				float fOod = 1.0f / vDirection[i];
				fT1	= (vBoxMin[i] - vVehiclePosition[i]) * fOod;
				fT2	= (vBoxMax[i] - vVehiclePosition[i]) * fOod;
				
				if (fT1 > fT2)
				{
					float fTemp	= fT1;
					fT1 = fT2;
					fT2	= fTemp;
				}
			
				fT = Mathf.Max (fT, fT1);
				fTMax = Mathf.Min (fTMax, fT2);
			
				if (fT > fTMax)
				{
					return false;
				}
			}
		}
		
		vInter = vVehiclePosition + (vDirection * fT);
		
		return true;
	}
	
	protected Intersection FindNextIntersectionWithSphere (Vector3 vVehiclePosition, Vector3 vFutureVehiclePosition, float fVehicleRad)
	{
		Intersection intersection = new Intersection(this);
		
		float 	fTempDist;
		Vector3	vTempPoint;
		Vector3	vDir = (vFutureVehiclePosition - vVehiclePosition).normalized;
		intersection.Intersect = IntersectRaySphere (vVehiclePosition, vDir, Position, 
		                                              fVehicleRad + ScaledRadius, out fTempDist, out vTempPoint);

		bool bIntersectsFurther = (vFutureVehiclePosition - vVehiclePosition).sqrMagnitude < (vTempPoint - vVehiclePosition).sqrMagnitude;

		if (intersection.Intersect && bIntersectsFurther)
		{
			intersection.Intersect = false;
		}

		intersection.Distance	= fTempDist;
		intersection.Point		= vTempPoint;
		
		return intersection;
	}
	
	protected bool IntersectRaySphere (Vector3 vPoint, Vector3 vDir, Vector3 vCenter, float fRad, out float fT, out Vector3 vInterPoint)
	{
		fT	= 0f;
		vInterPoint	= new Vector3();
		Vector3 m = vPoint - vCenter;
		float b = Vector3.Dot (m, vDir);
		float c = Vector3.Dot (m, m) - fRad * fRad;
		
		if (c > 0.0f && b > 0.0f)
		{
			return false;
		}
		
		float discr	= b * b - c;
		
		if (discr < 0.0f)
		{
			return false;
		}
		
		fT= -b - Mathf.Sqrt (discr);
		
		if (fT < 0.0f)
		{
			fT = 0.0f;
		}
		
		vInterPoint	= vPoint + fT * vDir;
		
		return true;
	}
	
	protected bool TestRaySphere (Vector3 vPoint, Vector3 vDir, Vector3 vCenter, float fRad)
	{
		Vector3 m = vPoint - vCenter;
		float c	= Vector3.Dot (m, m) - fRad * fRad;
		
		if (c <= 0f)
		{
			return true;
		}
		
		float b	= Vector3.Dot (m, vDir);
		
		if (b > 0f)
		{
			return false;
		}
		
		float disc	= b*b - c;
		
		if (disc < 0f)
		{
			return false;
		}
		
		return true;
	}
	
	/// <summary>
	/// Recalculates the object's scaled radius and center
	/// </summary>
	protected virtual void RecalculateScaledValues() 
	{
		if (Transform == null)
		{
			// Since this value gets assigned on Awake, we need to assign it when on the editor
			Transform = GetComponent<Transform>();
		}

		var scale  = Transform.lossyScale;
		_scaledRadius = _radius * Mathf.Max(scale.x, Mathf.Max(scale.y, scale.z));
		_scaledCenter = Vector3.Scale(_center, scale);
		_squaredRadius = _scaledRadius * _scaledRadius;

		if (DOColliderType.Capsule == _colliderType)
		{
			if (CapsuleDirection.XAxis == Direction)
			{
				_capHeight	= (Height * scale.x) - (2.0f * ScaledRadius);
			}
			else if (CapsuleDirection.YAxis == Direction)
			{
				_capHeight	= (Height * scale.y) - (2.0f * ScaledRadius);
			}
			else
			{
				_capHeight	= (Height * scale.z) - (2.0f * ScaledRadius);
			}

		}
	}	

	protected virtual void OnDrawGizmos()
	{
		if (_drawGizmos)
		{
			if (Transform == null)
			{
				// Since this value gets assigned on Awake, we need to assign it when on the editor
				Transform = GetComponent<Transform>();
			}
			Gizmos.color = Color.blue;
			Gizmos.DrawWireSphere(Position, ScaledRadius);
		}
	}
	#endregion
}

