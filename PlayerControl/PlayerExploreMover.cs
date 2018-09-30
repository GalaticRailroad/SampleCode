/***********************************************************************
 * 
 * The purpose of this class is to control movement of the player during 
 * the game.
 * 
 ***********************************************************************/

using UnityEngine;
using UnityEngine.Profiling;
using DarkTonic.PoolBoss;
using DarkTonic.MasterAudio;

public class PlayerExploreMover : FlyingTickedVehicle
{
    protected override void Awake()
    {
        base.Awake();

        m_transform = transform;
        m_inputManager = GameObject.FindGameObjectWithTag("Manager").GetComponent<InputManager>();
        m_playerShip = GameObject.FindGameObjectWithTag("Player").GetComponent<PlayerShip>();
        m_rigidbody = GetComponent<Rigidbody>();
        m_pShipTransform = m_playerShip.MGUIActorData.transform;
        m_radar = GameObject.FindGameObjectWithTag("PingRadar").GetComponent<PingRadar>();
        m_landingAbility = GetComponentInChildren<LandingAbility>();

        maxRate = m_rigidbody.maxAngularVelocity;
        m_vSavedInerTensor = m_rigidbody.inertiaTensor;
        m_qSavedInterTensorRot = m_rigidbody.inertiaTensorRotation;
        m_vSavedCenterOfMass = m_rigidbody.centerOfMass;
        m_collisionDamage.Setup(this, m_playerShip);
        m_vSavedThrust = thrust;
        m_fSavedDrag = m_rigidbody.drag;

        Quaternion axisRotation = Quaternion.AngleAxis(MTransform.localRotation.eulerAngles[0], rotateAround);
        minQuaternion = axisRotation * Quaternion.AngleAxis(m_fRotateMin, rotateAround);
        maxQuaternion = axisRotation * Quaternion.AngleAxis(m_fRotateMax, rotateAround);
        m_fRotateRange = m_fRotateMax - m_fRotateMin;
    }

    protected override void Start()
    {        
        SphereCollider col = MCollider as SphereCollider;

        //We manually set these values so we can have compound colliders without sacrificing precision
        m_rigidbody.centerOfMass = MTransform.InverseTransformPoint(MCollider.bounds.center);        
        m_rigidbody.inertiaTensor = new Vector3(col.radius * 2f, col.radius * 2f, col.radius * 2f);
        m_rigidbody.inertiaTensorRotation = Quaternion.Euler(0f, 0f, 0f);
    }

    /********************************************************************************************************************************************
	 * 
	 * Public functions
	 * 
	 ********************************************************************************************************************************************/

    public void HandleFixedUpdate()
    {
        if (!m_playerShip.IsDead() && !InputManager.Instance.InteractMode)
        {
            torques.x = new Vector2(shipExtents.y, shipExtents.z).magnitude * thrust.x;
            torques.y = new Vector2(shipExtents.x, shipExtents.z).magnitude * thrust.y; //normally would be x and z, but mesh is rotated 90 degrees in mine.  
            torques.z = new Vector2(shipExtents.x, shipExtents.y).magnitude * thrust.z; //normally would be x and y, but mesh is rotated 90 degrees in mine.

            Vector3 vTurnProp = GetOrientationProp();

            if (InputManager.Instance.NoviceMode)
            {
                vTurnProp.z = 0f;
            }

            if (!MLandingAbility.Landed)
                RCS(vTurnProp);

            if (vTurnProp == Vector3.zero && (!MLandingAbility.Landed && !MLandingAbility.Landing))
            {
                m_resetOrientationTimer.ModifyCurrent(Time.fixedDeltaTime);

                if (m_resetOrientationTimer.IsMaxed() && !m_bReOrientLock)
                {
                    Vector3 vLookDir = MTransform.forward;
                    ExperimentalOrientationChange(vLookDir, m_fAutoCorrectSpeed);
                }
            }
            else if (vTurnProp == Vector3.zero && LockOnCamera.Instance.MTargetCast.LockedTarget != null)
            {
                Vector3 vLookDir = MTransform.forward;
                ExperimentalOrientationChange(vLookDir, m_fAutoCorrectSpeed);
            }

            else if (PointToTurnToward != Vector3.zero)
            {
                ExperimentalOrientationChange(PointToTurnToward, m_fAutoCorrectSpeed);
            }
            else
            {
                m_resetOrientationTimer.SetToMin();
                m_bReOrientLock = false;
            }


            if (Transform.position.y < 50f)
            {
                Vector3 vPos = MTransform.position;
                vPos.y = 55f;
                Rigidbody.MovePosition(vPos);
            }

            MLandingAbility.ContinueLanding();

            UpdateMovementSoundLogic();

            m_vPrevInputValues = vTurnProp;
        }

        if (Rigidbody.drag < 0f)
        {
            Rigidbody.drag = m_fSavedDrag;
        }
    }

    //InputManager calls this to force player realignment
    public void AlignUpright()
    {
        if ((!MLandingAbility.Landed || !MLandingAbility.Landing) && !m_bReOrientLock)
        {
            m_resetOrientationTimer.SetToMax();
        }
    }

    public void ExperimentalOrientationChange(Vector3 vDir, float fMaxDegrees)
    {      
        Quaternion desiredRotation = Quaternion.LookRotation(vDir);
        desiredRotation = Quaternion.RotateTowards(Transform.rotation, desiredRotation, fMaxDegrees);

        float kp = (6f * m_fFrequency) * (6f * m_fFrequency) * 0.25f;
        float kd = 4.5f * m_fFrequency * m_fDamping;
        float dt = Time.fixedDeltaTime;
        float g = 1 / (1 + kd * dt + kp * dt * dt);
        float ksg = kp * g;
        float kdg = (kd + kp * dt) * g;
        Vector3 x;
        float xMag;
        Quaternion q = desiredRotation * Quaternion.Inverse(MTransform.rotation);
        q.ToAngleAxis(out xMag, out x);
        x.Normalize();
        x *= Mathf.Deg2Rad;
        Vector3 pidv = kp * x * xMag - kd * Rigidbody.angularVelocity;
        Quaternion rotInertia2World = Rigidbody.inertiaTensorRotation * MTransform.rotation;
        pidv = Quaternion.Inverse(rotInertia2World) * pidv;
        pidv.Scale(Rigidbody.inertiaTensor);
        pidv = rotInertia2World * pidv;

        bool bNaNCheck = float.IsNaN(pidv.x) || float.IsNaN(pidv.y) || float.IsNaN(pidv.z);
        if (!bNaNCheck)
        {
            if (pidv.magnitude > fMaxDegrees)
            {
                pidv = pidv.normalized * fMaxDegrees;
            }

            Rigidbody.AddTorque(pidv);

            if (pidv.sqrMagnitude <= 0.1f)
            {
                m_bReOrientLock = true;
            }
        }

        if (InputManager.Instance.IsMovement())
        {
            SpeedStat.Accelerate();

            if (MTransform.forward.y < 0f)
            {
                Vector3 vDownForce = Physics.gravity.y * GameSimulation.Instance.GravityMultiplier * MTransform.forward.y * MTransform.forward * GameSimulation.Instance.GravityMultiplier;
                m_rigidbody.AddForce(vDownForce, ForceMode.Force);
            }
        }
        else
        {
            SpeedStat.Decelerate();
        }

        if (!MLandingAbility.Landed)
            SpeedStat.AddForce(0f, 0f);

        bool bDashCheck = m_playerShip.MDashAbility == null ||
            (m_playerShip.MDashAbility != null && m_playerShip.MDashAbility.Cooldown.IsMin()) ||
            (m_playerShip.MSpeedGateEffect.ForcesActive());

        if ((Rigidbody.velocity.magnitude > 0 && bDashCheck) || DisableAeroDynamic)
        {
            AeroDynamicEffect.ModifyCurrent(Time.fixedDeltaTime * m_fAeroGainMod);
            // compare the direction we're pointing with the direction we're moving:
            m_AeroFactor = Vector3.Dot(MTransform.forward, Rigidbody.velocity.normalized);
            // multipled by itself results in a desirable rolloff curve of the effect
            m_AeroFactor *= m_AeroFactor;
            // Finally we calculate a new velocity by bending the current velocity direction towards
            // the the direction the plane is facing, by an amount based on this aeroFactor
            Vector3 newVelocity = Vector3.Lerp(Rigidbody.velocity, MTransform.forward * Rigidbody.velocity.magnitude,
                m_AeroFactor * Rigidbody.velocity.magnitude * AeroDynamicEffect.Current * Time.fixedDeltaTime);

            Rigidbody.velocity = newVelocity;
        }
        else
        {
            AeroDynamicEffect.SetToMin();
        }
    }

    public bool QuaternionsEqual(Quaternion q1, Quaternion q2)
    {
        return (q1.Equals(q2) || (q1 == q2));
    }

    public void HandleRotations()
    {
        ExecuteRotation();

        if (m_inputManager.IsMovement())
        {
            float fTemp = m_fRollAmount;
            if (Mathf.Abs(m_fRollAmount) < Mathf.Abs(m_fTurnAmount))
            {
                fTemp = -m_fTurnAmount;
            }

            m_pShipTransform.localRotation = Quaternion.Lerp(m_pShipTransform.localRotation,
                                                             Quaternion.Euler(-(m_fPitchAmount * 0.1f), m_fTurnAmount * 0.1f, fTemp * 0.35f),
                                                             Time.fixedDeltaTime * m_fRotDamping);
        }
        else
        {
            m_pShipTransform.localRotation = Quaternion.Lerp(m_pShipTransform.localRotation,
                                                             Quaternion.Euler(0f, 0f, 0f),
                                                             Time.fixedDeltaTime * m_fRotDamping);
        }
    }

    public void HandleForce()
    {
        //No active ship movement
        if (!InputManager.Instance.IsMovement())
        {
            m_playerShip.Speed.Decelerate();
            m_rigidbody.velocity = (m_transform.forward * m_playerShip.Speed.CurrentSpeed) + CollisionVelocity;
        }
        else
        {
            //Ship needs to do some kind of match speed
            if (InputManager.Instance.IsMovingOneTouch())
            {
                m_playerShip.Speed.Accelerate();

                if (ActorType.Player == m_radar.LockedOn.Type)
                {
                    m_rigidbody.velocity = (m_transform.forward * m_playerShip.Speed.CurrentSpeed) + CollisionVelocity;
                }
                else
                {
                    m_rigidbody.velocity = (m_transform.forward * m_playerShip.Speed.CurrentSpeed) + CollisionVelocity;
                }
            }
            //Normal ship movement
            else
            {
                m_playerShip.Speed.Accelerate();
                m_rigidbody.velocity = (m_transform.forward * m_playerShip.Speed.CurrentSpeed) + CollisionVelocity;
            }
        }
    }

    public void HandleInput()
    {
        //This is nasty, but we will miss it during fixed update if we do it there.
        RollCorrection();
    }

    public void HandleTorqueChange(float fPercentage)
    {
        thrust = m_vSavedThrust * fPercentage;
    }

    /* For cases where we have the following number line
	 * 0	1	 0
	 * |---------|
	 * val : the value we want to get the proportion of
	 * start : starting value of the number line
	 * end : ending value of the number line
	 * mod : changes the ending value of the start and end (mod = 2.0 equals 0.5 instead of 0 for example)
	 */
    public float GetPeakNumber(float val, float start, float end, float center, float mod)
    {
        if (mod != 0.0)
        {
            if (val > center)
            {
                return (1.0f - ((val - center) / (end - center)) / mod);
            }

            return ((1.0f / mod) - val) + ((val - start) / (center - start));
        }

        if (val > center)
        {
            return (1.0f - ((val - center) / (end - center)));
        }

        return ((val - start) / (center - start));
    }

    public float GetCurrentPropBoost()
    {
        return 1.0f;
    }

    public bool IsBraking()
    {
        return SpeedStat.IsBraking;
    }

    public Vector3 GetPosition()
    {
        return m_transform.position;
    }

    public void SetPosition(Vector3 vPos, Quaternion rotation)
    {
        Rigidbody.isKinematic = true;
        m_transform.position = vPos;
        m_transform.rotation = rotation;

        m_radar.CleanRadar();
        Rigidbody.isKinematic = false;
    }

    public Vector3 GetOrientProportion()
    {
        return GetOrientationProp();
    }

    public override void UpdateOrientationVelocity(Vector3 velocity)
    {
        Speed = SpeedStat.CurrentSpeed;
        OrientationVelocity = Mathf.Approximately(SpeedStat.CurrentSpeed, 0) ? Transform.forward : velocity / SpeedStat.CurrentSpeed;
    }

    /** These are set whenever a collider is added, but not recalculated when one is removed
	 */
    public void ResetIntertiaTensors()
    {
        Rigidbody.inertiaTensor = m_vSavedInerTensor;
        Rigidbody.inertiaTensorRotation = m_qSavedInterTensorRot;
        Rigidbody.centerOfMass = m_vSavedCenterOfMass;
    }

    public bool AllowTorquing
    {
        get { return m_bAllowTorquing; }
        set { m_bAllowTorquing = value; }
    }

    public float TurnProportion
    {
        get { return m_fTurnProp; }
    }

    public float PitchProportion
    {
        get { return m_fPitchProp; }
    }

    public float MaxTurnSpeed
    {
        get { return m_fTurnSpeed; }
    }

    public float MaxPitchSpeed
    {
        get { return m_fPitchSpeed; }
    }

    public LandingAbility MLandingAbility
    {
        get
        {
            if (null == m_landingAbility)
            {
                m_landingAbility = GetComponentInChildren<LandingAbility>();
            }

            return m_landingAbility;
        }
    }

    public bool DisableAeroDynamic
    {
        get { return m_bDisableAero; }
        set { m_bDisableAero = value; }
    }

    public Transform MTransform
    {
        get
        {
            if (null == m_transform)
            {
                m_transform = transform;
            }

            return m_transform;
        }
    }

    public GameObject MGameObject
    {
        get
        {
            if (null == m_gameObject)
            {
                m_gameObject = gameObject;
            }

            return m_gameObject;
        }
    }

    public Collider MCollider
    {
        get
        {
            if (null == m_collider)
            {
                m_collider = GetComponent<Collider>();
            }

            return m_collider;
        }
    }

    public AudioSource MAudioSource
    {
        get
        {
            if (null == m_source)
            {
                m_source = gameObject.AddComponent<AudioSource>();
            }

            return m_source;
        }
    }

    public StatisticSpeed SpeedStat
    {
        get { return m_playerShip.Speed; }
    }

    public override float Speed
    {
        get { return m_playerShip.Speed.CurrentSpeed; }
        set
        {
            m_playerShip.Speed.CurrentSpeed = (Mathf.Clamp(value, 0, m_playerShip.Speed.Max));
            DesiredSpeed = m_playerShip.Speed.Current;
        }
    }

    /// <summary>
    /// Current vehicle velocity
    /// </summary>
    public override Vector3 Velocity
    {
        get
        {
            return Rigidbody.velocity;
        }
        set
        {
            Rigidbody.velocity = value;
        }
    }

    public override Vector3 DesiredVelocity
    {
        get
        {
            return m_transform.forward * DesiredSpeed;
        }
        protected set
        {
            base.DesiredVelocity = value;
        }
    }

    public ValueRange AeroDynamicEffect
    {
        get { return m_AerodynamicEffect; }
    }

    public Vector3 LastCollisionVector
    {
        get;
        set;
    }

    public Vector3 PointToTurnToward
    {
        get { return m_vPointToTurnToward; }
        set { m_vPointToTurnToward = value; }
    }

    /********************************************************************************************************************************************
	 * 
	 * Protected functions
	 * 
	 ********************************************************************************************************************************************/

    protected override void OnEnable()
    {
    }

    protected override void CalculateForces()
    {
        if (!CanMove || MaxForce == 0 || MaxSpeed == 0)
        {
            return;
        }

        Speed = SpeedStat.CurrentSpeed;

        ExecuteRotation();

        Vector3 force = Transform.forward * Speed;

        if (IsPlanar)
        {
            force.y = 0;
        }
        LastRawForce = force;

        Vector3 newVelocity = Vector3.ClampMagnitude(force, SpeedStat.Current);
        DesiredVelocity = newVelocity;
        Vector3 adjustedVelocity = Vector3.zero;
        adjustedVelocity = SteerAI.CalculatePostProcessSteerings(DesiredVelocity);

        if (adjustedVelocity != Vector3.zero)
        {
            adjustedVelocity = Vector3.ClampMagnitude(adjustedVelocity, SpeedStat.Current);
            TraceDisplacement(adjustedVelocity, Color.cyan);
            TraceDisplacement(newVelocity, Color.white);
            newVelocity = adjustedVelocity;
        }

        UpdateOrientationVelocity(newVelocity);
    }

    protected override void AdjustOrientation(float deltaTime)
    {
        Profiler.BeginSample("AdustOrientation");
        /* 
		 * Avoid adjusting if we aren't applying any velocity. We also
		 * disregard very small velocities, to avoid jittery movement on
		 * rounding errors.
		 */
        if (DesiredSpeed > MinSpeedForTurning && Velocity != Vector3.zero)
        {
            var newForward = OrientationVelocity;
            if (IsPlanar)
            {
                newForward.y = 0;
                newForward.Normalize();
            }

            if (TurnTime != 0)
            {
                newForward = Vector3.Slerp(Transform.forward, newForward, deltaTime / TurnTime);
            }
            Transform.forward = newForward;
        }

        if (m_inputManager.IsMovement())
        {
            m_pShipTransform.localRotation = Quaternion.Lerp(m_pShipTransform.localRotation,
                                                             Quaternion.Euler(-(m_fPitchAmount), m_fTurnAmount * 0.5f, m_fRollAmount * 0.5f),
                                                             Time.fixedDeltaTime * m_fRotDamping);
        }
        else
        {
            m_pShipTransform.localRotation = Quaternion.Lerp(m_pShipTransform.localRotation,
                                                             Quaternion.Euler(0f, 0f, 0f),
                                                             Time.fixedDeltaTime * m_fRotDamping);
        }

        Profiler.EndSample();
    }

    protected Vector3 SanityCheckInput (Vector3 vInput)
    {
        Vector3 vResult = vInput;

        float fValue = Mathf.MoveTowards(m_vPrevInputValues.y, vInput.y, 1f * Time.deltaTime);

        if (Mathf.Abs(m_vPrevInputValues.y - vInput.y) > 0.2f)
        {
            vResult.y = fValue;
        }

        return vResult;
    }

	protected void RCS (Vector3 vTurnProp)
	{
		//angular acceleration = torque/mass
		rates 				= torques / m_rigidbody.mass;

		Vector3	vThrustProp	= vTurnProp;

//		//determine targer rates of rotation based on user input as a percentage of the maximum angular velocity
		m_vTargetVelocity 	= new Vector3(vThrustProp.x*rates.x,vThrustProp.y*rates.y,vThrustProp.z*rates.z);

//		//take the rigidbody.angularVelocity and convert it to local space; we need this for comparison to target rotation velocities

		curVelocity 		= MTransform.InverseTransformDirection(m_rigidbody.angularVelocity);

		//****************************************************************************************************************
		//For each axis:  If the ship's current rate of rotation does not equal the desired rate of rotation, first check to see
		//if it is a matter of drift or "jittering", which is when it keeps jumping from positive to negative to positive thrust because the
		//values are so close to zero (to see what I mean, set  snapThreshold = 0, rotate the ship on multiple axes, then let it try
		//to come to a complete stop.  It won't.)  If it is just drift/jittering, turn off the thrust for the axis, and just set the current 
		//angular velocity to the target angular velocity.  Otherwise, the user is still giving input, and we haven't reached the 
		//desired rate of rotation.  In that case, we set the axis activation value = to the direction in which we need thrust.
		//****************************************************************************************************************
		
		if(curVelocity.x != m_vTargetVelocity.x)
			if(Mathf.Abs(m_vTargetVelocity.x - curVelocity.x) < rates.x*Time.fixedDeltaTime*snapThreshold)
		{
			tActivation.x = 0;
			curVelocity.x = m_vTargetVelocity.x;
		}
		else
			tActivation.x = Mathf.Sign(m_vTargetVelocity.x-curVelocity.x);
		
		if(curVelocity.y != m_vTargetVelocity.y)
			if(Mathf.Abs(m_vTargetVelocity.y - curVelocity.y) < rates.y*Time.fixedDeltaTime*snapThreshold)
		{
			tActivation.y = 0;
			curVelocity.y = m_vTargetVelocity.y;
		}
		else
			tActivation.y = Mathf.Sign(m_vTargetVelocity.y-curVelocity.y);
		
		if(curVelocity.z != m_vTargetVelocity.z)
			if(Mathf.Abs(m_vTargetVelocity.z - curVelocity.z) < rates.z*Time.fixedDeltaTime*snapThreshold)
		{
			tActivation.z = 0;
			curVelocity.z = m_vTargetVelocity.z;
		}
		else
			tActivation.z = Mathf.Sign(m_vTargetVelocity.z-curVelocity.z);

        //here, we manually set the rigidbody.angular velocity to the value of our current velocity.
        //this is done to effect the manual changes we may have made on any number of axes.
        //if we didn't do this, the jittering would continue to occur.

        if (InputManager.Instance.NoviceMode)
        {

        }
        else
        {
            m_rigidbody.angularVelocity = MTransform.TransformDirection(curVelocity);
        }

		//call the function that actually handles applying the torque
		FireThrusters(vTurnProp);
	}

	protected void FireThrusters(Vector3 vOrientationProp)
	{
		//for each axis, applies torque based on the torque available to the axis in the direction indicated by the activation value.
		//-1 means we are applying torque to effect a negative rotation.  +1 does just the opposite.  0 means no torque is needed.
		Vector3	vThrustProp	= vOrientationProp;
        Vector3 vXTorque = Vector3.zero;
        Vector3 vYTorque = Vector3.zero;
        Vector3 vZTorque = Vector3.zero;

        if (!InputManager.Instance.NoviceMode)
        {
            vXTorque = tActivation.x * MTransform.TransformDirection(Vector3.right) * torques.x * Mathf.Abs(vThrustProp.x) * Time.fixedDeltaTime;
            vYTorque = tActivation.y * MTransform.TransformDirection(Vector3.up) * torques.y * Mathf.Abs(vThrustProp.y) * Time.fixedDeltaTime;
            vZTorque = tActivation.z * MTransform.TransformDirection(Vector3.forward) * torques.z * Mathf.Abs(vThrustProp.z) * Time.fixedDeltaTime;
        }
        else
        {
            Vector3 vLocalRight = MTransform.TransformDirection(Vector3.right);
            vLocalRight.y = 0;            
            
            vXTorque = tActivation.x * vLocalRight.normalized * torques.x * Mathf.Abs(vThrustProp.x) * Time.fixedDeltaTime;
            vYTorque = tActivation.y * (Vector3.up) * torques.y * Mathf.Abs(vThrustProp.y) * Time.fixedDeltaTime;
        }

        if (!MLandingAbility.Landed)
		{
			Vector3 localVelocity 	= MTransform.InverseTransformDirection(m_rigidbody.velocity);
			float	ForwardSpeed 	= Mathf.Max(0, localVelocity.z);

			Vector3 vTorque			= Vector3.zero;

            if (InputManager.Instance.NoviceMode)
            {
                Quaternion localRotation = MTransform.localRotation;
                Quaternion axisRotation = Quaternion.AngleAxis(localRotation.eulerAngles[0], rotateAround);

                float angleFromMin = Quaternion.Angle(axisRotation, minQuaternion);
                float angleFromMax = Quaternion.Angle(axisRotation, maxQuaternion);

                float fDotUp = Vector3.Dot(MTransform.forward, Vector3.up);
                float fDotDown = Vector3.Dot(MTransform.forward, -Vector3.up);
                bool bDiffUp = (1f - fDotUp) < 0.1f;
                bool bDiffDown = (1f - fDotDown) < 0.1f;
                bool bDiff = bDiffUp || bDiffDown;
                bool bDownDirMovement = bDiffUp && !bDiffDown && tActivation.x > 0;
                bool bUpDirMovement = bDiffDown && !bDiffUp && tActivation.x < 0;                

                if (!bDiff || bDownDirMovement || bUpDirMovement)
                {
                    if (tActivation.x != 0)
                        vTorque += vXTorque;
                }
                else
                {
                    Vector3 vAngular = m_rigidbody.angularVelocity;
                    vAngular.x = 0f;
                    vAngular.z = 0f;
                    m_rigidbody.angularVelocity = vAngular;
                }

                if (tActivation.y != 0)
                    vTorque += vYTorque;
            }
            else
            {
                if (tActivation.x != 0)
                    vTorque += vXTorque;
                if (tActivation.y != 0)
                    vTorque += vYTorque;
                if (tActivation.z != 0)
                    vTorque += vZTorque;
            }

			if (AllowTorquing && !InputManager.Instance.NoviceMode)
			{
				m_rigidbody.AddTorque(vTorque);
			}
            else if (AllowTorquing && InputManager.Instance.NoviceMode)
            {
                m_rigidbody.AddTorque(vTorque);
            }
		}

		if (InputManager.Instance.IsMovement() && !m_playerShip.m_airBrakePress.IsActive)
		{
			SpeedStat.Accelerate();

			if (MTransform.forward.y < 0f)
			{
				Vector3 vDownForce = Physics.gravity.y * GameSimulation.Instance.GravityMultiplier * MTransform.forward.y * MTransform.forward * GameSimulation.Instance.GravityMultiplier;
				m_rigidbody.AddForce(vDownForce, ForceMode.Force);
			}
		}
		else
		{
			SpeedStat.Decelerate();
		}

		if (!MLandingAbility.Landed)
			SpeedStat.AddForce(vThrustProp.x, vThrustProp.y);

        bool bDashCheck = m_playerShip.MDashAbility == null || 
            (m_playerShip.MDashAbility != null && m_playerShip.MDashAbility.Cooldown.IsMin()) ||
            (m_playerShip.MSpeedGateEffect.ForcesActive()); 

        if (m_bAeroSteering)
        {
            if ((Rigidbody.velocity.magnitude > 0 && bDashCheck) || DisableAeroDynamic)
            {
                AeroDynamicEffect.ModifyCurrent(Time.fixedDeltaTime * m_fAeroGainMod);
                // compare the direction we're pointing with the direction we're moving:
                m_AeroFactor = Vector3.Dot(MTransform.forward, Rigidbody.velocity.normalized);
                // multipled by itself results in a desirable rolloff curve of the effect
                m_AeroFactor *= m_AeroFactor;
                // Finally we calculate a new velocity by bending the current velocity direction towards
                // the the direction the plane is facing, by an amount based on this aeroFactor
                Vector3 newVelocity = Vector3.Lerp(Rigidbody.velocity, MTransform.forward * Rigidbody.velocity.magnitude,
                    m_AeroFactor * Rigidbody.velocity.magnitude * AeroDynamicEffect.Current * Time.fixedDeltaTime);

                Rigidbody.velocity = newVelocity;
            }
            else
            {
                AeroDynamicEffect.SetToMin();
            }
        }
	}

	protected Vector3 GetOrientationProp ()
	{
		Vector3 vTorqueProp				= new Vector3();
		Vector2 vSavedPropLeft			= InputManager.Instance.GetSavedTouchProp(InputSide.Left);
		Vector2 vSavedPropRight			= InputManager.Instance.GetSavedTouchProp(InputSide.Right);
		
		float fTotalDistFromCenter 		= (vSavedPropLeft.y - 0.5f) + (vSavedPropRight.y - 0.5f);
		float fTotalDistFromEachOther 	= vSavedPropLeft.y - vSavedPropRight.y;
		
		if (InputManager.Instance.IsMouseActive)
		{
			vTorqueProp			= m_inputManager.GetMovementAxis();
		}
		else
		{
			if (InputManager.Instance.IsMovement())
			{
				m_fPitchProp		= (ModifyProportion(Mathf.Abs(fTotalDistFromCenter), 
				                      	Mathf.Sign(fTotalDistFromCenter)));
				m_fTurnProp			= ModifyProportion(Mathf.Abs(fTotalDistFromEachOther),
				                        Mathf.Sign (fTotalDistFromEachOther));
				HandleRoll();

				vTorqueProp			= new Vector3(m_fPitchProp, m_fTurnProp, m_fRollProp);
			}
		}

		return vTorqueProp;
	}

	void OnCollisionEnter (Collision collision)
	{
		m_playerShip.OnCollisionHitForAbilities(collision);
		MLandingAbility.MoverCollisionHitLogic(collision);
        LockOnCamera.Instance.PlayShakeEvent(m_collideEventData);

		for (int i = 0; i < collision.contacts.Length; ++i)
		{
			Vector3 	vLoc 	= collision.contacts[i].point;
			Collider	hitCol	= collision.contacts[i].thisCollider;		

			if (hitCol == GetComponent<Collider>())
			{
				if (MLandingAbility.Landing || MLandingAbility.Landed)
				{
					return;
				}

                bool bCollideDamageLayer = LayerMask.NameToLayer("Scenery") == collision.collider.gameObject.layer ||
                                            LayerMask.NameToLayer("Building") == collision.collider.gameObject.layer;
                //PlayEffect
                if (bCollideDamageLayer)
				{
					m_collisionDamage.SetCollisionInfo(collision);
					m_playerShip.HandleMod(m_collisionDamage);
					m_playerShip.StealthMeter.ModifyCurrent(-Rigidbody.velocity.magnitude);
                    m_speedDamageMod.Amount = -m_playerShip.m_fSpeed;                    
                    LastCollisionVector = collision.impulse / Time.fixedDeltaTime;
                    collision.gameObject.SendMessage("OnRammedByPlayer", m_speedDamageMod, SendMessageOptions.DontRequireReceiver);

                    MasterAudio.PlaySound(m_sOnRegularHit);
				}
				else if (LayerMask.NameToLayer("Water") == collision.collider.gameObject.layer)
				{
					m_playerShip.StealthMeter.ModifyCurrent(-Rigidbody.velocity.magnitude);
                    Vector3 vAdded = m_playerShip.MTransform.forward;
                    vAdded.y = 0;
                    vAdded.Normalize();
                    vAdded *= 4f;
                    PoolBoss.Spawn(m_waterCollisionPrefab.transform, vLoc + vAdded, Quaternion.identity, null);
				}
				else if (LayerMask.NameToLayer("Actor") == collision.collider.gameObject.layer)
				{
					m_playerShip.StealthMeter.ModifyCurrent((m_collisionDamage.Amount));
				}
                else if (LayerMask.NameToLayer("FlockingSpirits") == collision.collider.gameObject.layer)
                {
                    MasterAudio.PlaySound(m_sOnFlockingHit);

                    Transform mForm = PoolBoss.Spawn(m_impactEffect.transform, vLoc, Quaternion.identity, MTransform);
                    mForm.parent = MTransform;
                    mForm.localPosition = MTransform.InverseTransformPoint(vLoc);
                }
                else if (LayerMask.NameToLayer("Vehicle") == collision.collider.gameObject.layer)
                {
                    MasterAudio.PlaySound(m_sOnVehicleHit);

                    Transform mForm = PoolBoss.Spawn(m_impactEffect.transform, vLoc, Quaternion.identity, MTransform);
                    mForm.parent = MTransform;
                    mForm.localPosition = MTransform.InverseTransformPoint(vLoc);
                }
            }
		}
	}

	void OnCollisionExit (Collision collision)
	{
		MLandingAbility.MoverCollisionExitLogic(collision);
	}

	protected void OnStaticSceneryHit (ContactPoint cPoint)
	{
		float fLandingAngle	= Vector3.Dot (cPoint.normal, MTransform.forward);

		if (fLandingAngle < m_fFrontCollisionTol)
		{
			Vector3 vReflect 	= Vector3.Reflect(MTransform.forward, cPoint.normal);
			Vector3 vCross 		= Vector3.Cross(MTransform.forward, vReflect);
			float 	fDot		= Vector3.Dot (MTransform.forward, cPoint.normal);
			float 	fProp 		= Mathf.Pow (fDot, 3f) * 8f;

			Rigidbody.AddRelativeTorque(vCross * SpeedStat.AccelForce.Current * fProp, ForceMode.Force);
			SpeedStat.AccelForce.SetPercentOfCurrent(0.65f);
		}
	}

	protected void UpdateMovementSoundLogic ()
	{
		if (m_auPlayerMove != null)
		{
			MAudioSource.loop = true;
			MAudioSource.clip = m_auPlayerMove;

			if (!MAudioSource.isPlaying)
			{
				MAudioSource.Play();
			}

			if (!MLandingAbility.Landed)
			{
				if (!InputManager.Instance.IsMovement())
				{				
					MAudioSource.pitch = 0.3f + (SpeedStat.AccelForce.SavedMaxProprotion() * 4.0f);

					Vector3 vTorqueProp = GetOrientationProp();
					MAudioSource.pitch	+= ((Mathf.Abs(vTorqueProp.x) + Mathf.Abs(vTorqueProp.y) + Mathf.Abs(vTorqueProp.z))) / 3.0f;
				}
			}
			else
			{
				MAudioSource.pitch = Mathf.Lerp(MAudioSource.pitch, 0f, Time.deltaTime);
			}
		}
	}

	/********************************************************************************************************************************************
	 * 
	 * Private Functions
	 * 
	 ********************************************************************************************************************************************/ 

	private void HandleRoll ()
	{
		float fValue = 0f;

		if (m_inputManager.IsMouseActive)
		{
			m_rollStatus 	= m_inputManager.MRollStatus;

			if (RollStatus.Correct == m_rollStatus)
			{
				fValue = 1.0f;
			}
			else if (RollStatus.Left == m_rollStatus)
			{
				fValue = 1.0f;
			}
			else if (RollStatus.Right == m_rollStatus)
			{
				fValue = -1.0f;
			}
			else
			{
				fValue = 0.0f;
			}

			m_fRollProp = RollProportion(fValue);
		}
		else
		{
			Vector2 vSavedTouchPropRight 	= m_inputManager.GetSavedTouchProp(InputSide.Right);
			Vector2 vSavedTouchPropLeft		= m_inputManager.GetSavedTouchProp(InputSide.Left);
			bool 	bLeftDeadZone 			= vSavedTouchPropLeft.x > m_inputManager.RollDeadZone;
			bool 	bRightDeadZone			= (1.0f - vSavedTouchPropRight.x) > m_inputManager.RollDeadZone;

			if (bLeftDeadZone && bRightDeadZone)
			{
				m_rollStatus 	= RollStatus.Correct;
				fValue 			= 1.0f;
				m_fRollProp 	= RollProportion(fValue);
			}
			else if (bLeftDeadZone)
			{
				m_rollStatus 	= RollStatus.Left;
				fValue 			= vSavedTouchPropLeft.x;
				m_fRollProp 	= -RollProportion(fValue);
			}
			else if (bRightDeadZone)
			{
				m_rollStatus 	= RollStatus.Right;
				fValue 			= (1.0f - vSavedTouchPropRight.x);
				m_fRollProp 	= RollProportion(fValue);
			}
			else
			{
				m_rollStatus 	= RollStatus.None;
				fValue			= 0f;
				m_fRollProp 	= RollProportion(fValue);
			}
		}
	}

	private float RollProportion (float fValue)
	{
//		return Mathf.Clamp((64.0f * (Mathf.Pow (fValue, 3f))), -1f, 1f);
		return Mathf.Clamp(4.5f * fValue, -1f, 1f);
	}

	private void ExecuteRotation ()
	{
		float 		fRX 		= (m_fPitchAmount * Time.fixedDeltaTime);
		float 		fRY 		= (m_fTurnAmount * Time.fixedDeltaTime);
		float 		fRZ 		= (m_fRollAmount * Time.fixedDeltaTime);

		if (RollStatus.Correct != m_rollStatus)
		{
			m_transform.Rotate(-Vector3.right * fRX);
			m_transform.Rotate(Vector3.up * fRY);
			m_transform.Rotate(Vector3.forward * fRZ);
		}
		else
		{
			Quaternion targetRot 	= Quaternion.LookRotation(m_transform.forward);
			float fDegrees			= (targetRot.eulerAngles - m_transform.rotation.eulerAngles).magnitude;
			float fTotalTime		= fDegrees / m_fRollAmount;
			
			if (m_fRollCorrectCurrentTime < 1f)
			{
				m_transform.rotation		= Quaternion.Lerp(m_transform.rotation, targetRot, m_fRollCorrectCurrentTime);
				m_fRollCorrectCurrentTime 	+= Time.fixedDeltaTime / fTotalTime;
			}
			else
			{
				m_transform.rotation = targetRot;
			}
		}
	}

	private void RollCorrection ()
	{
		bool bPrevNotCorrecting = RollStatus.Correct != m_inputManager.MPrevRollStatus;
		bool bNowCorrecting		= RollStatus.Correct != m_inputManager.MRollStatus;

		//started
		if (bPrevNotCorrecting && bNowCorrecting)
		{
			m_fRollCorrectCurrentTime 	= 0f;
		}
		//finished
		else if (!bPrevNotCorrecting && !bNowCorrecting)
		{
			m_fRollAmount				= 0f;
		}
	}

	private float ModifyProportion (float proportion, float fSign)
	{
		float fTemp = 0f;

		if (!m_inputManager.IsMouseActive)
		{
			if (proportion > 0.8f)
			{
				fTemp = 1.0f;
			}
			else
			{
//				fTemp = 1.55f * Mathf.Pow(proportion, 2f);
				fTemp = 1.25f*proportion;
			}
		}
		else
		{
			fTemp = (Mathf.Pow(proportion, 2.0f));
		}
		
		return fSign * fTemp;
	}

    /********************************************************************************************************************************************
	 * 
	 * Public Variables
	 * 
	 ********************************************************************************************************************************************/

    [SoundGroupAttribute]
    public string m_sOnVehicleHit;
    [SoundGroupAttribute]
    public string m_sOnFlockingHit;
    [SoundGroupAttribute]
    public string m_sOnRegularHit;

	public float 		m_fTurnSpeed	= 60f;	// Degrees per second
	public float 		m_fPitchSpeed	= 35f;	// Degrees per second
	public float 		m_fRollSpeed	= 35f;	// Degrees per second
	public float 		m_fRotDamping	= 10f;
    public float        m_fDotUpwardThreshold = 0.95f;

    public float m_fFrequency = 1f;
    public float m_fDamping = 1f;

    public float m_fAutoCorrectSpeed = 60f;
    public ValueRange m_resetOrientationTimer;
    public bool m_bAeroSteering = true;    

    public Vector3 		thrust;					//Total thrust per axis
	public Vector3		shipExtents;
	public float 		snapThreshold;			//used to tweak how smoothly it will stabilize; .01 is probably as low as you want to go; lower is smoother, but more drift/jittering may occur.  I use 1.
    public float m_fSnapThresholdTurning;
    public float m_fSnapThresholdNotTurning;
	public float 		m_fFrontCollisionTol = 0.3f;
    public float m_fAeroGainMod = 1f;

	public CollisionDamageMod	m_collisionDamage;
    public GameObject m_impactEffect;

	public PEffectCode	m_effSceneryCol;
	public GameObject	m_waterCollisionPrefab;

	public AudioClip	m_auPlayerMove;
	public AudioClip    m_auPlayerBrake;

    public ShakeTransformEventData m_collideEventData;

    /********************************************************************************************************************************************
	 * 
	 * Protected Variables
	 * 
	 ********************************************************************************************************************************************/

    protected Transform 		m_transform;
	protected LandingAbility	m_landingAbility;
	protected Rigidbody			m_rigidbody;
	protected Collider			m_collider;
	protected Vector3			m_vSavedThrust;
    protected Vector3           m_vPointToTurnToward;
    protected float             m_fSavedDrag;
    protected bool              m_bDisableAero;
    protected bool              m_bReOrientLock;
    [SerializeField]
    protected Modification      m_speedDamageMod = new Modification();

    /********************************************************************************************************************************************
	 * 
	 * Private Variables
	 * 
	 ********************************************************************************************************************************************/

    [SerializeField] private ValueRange m_AerodynamicEffect = new ValueRange();   // How much aerodynamics affect the speed of the aeroplane.

    private float 					m_fTurnProp				= 0f;
	private float					m_fPitchProp			= 0f;
	private float 					m_fRollProp				= 0f;
	private float 					m_fTurnAmount			= 0f;
	private float 					m_fPitchAmount			= 0f;
	private float 					m_fRollAmount			= 0f;
	private bool					m_bAllowTorquing		= true;

	private Transform 				m_pShipTransform;
	private PlayerShip				m_playerShip;
	private RollStatus				m_rollStatus;
	private GameObject				m_gameObject;
	private float 					m_fRollCorrectCurrentTime;

	private InputManager			m_inputManager;
	private PingRadar				m_radar;
	public AudioSource				m_source;

	private Vector3					m_vSavedInerTensor;
	private Quaternion				m_qSavedInterTensorRot;
	private	Vector3					m_vSavedCenterOfMass;
	private Vector3 				m_vTargetVelocity;							//user input determines how fast user wants ship to rotate
	private Vector3 				tActivation = Vector3.zero;			//switch vector to indicate which axes need thrust and in which direction (values are: -1, 0, or 1)
	private Vector3 				inputs;									//just holds input axis values
	private Vector3 				torques;									//the amount of torque available for each axis, based on thrust
	private Vector3 				rates;										//the rates of angular acceleration for each axis, based on the torque available and ship mass
	private float 					maxRate;										//just holds the Physics.angularVelocity value; you can use other values here (for example: different ships may have different max rotation rates based on integrity of it's hull)
	private Vector3 				curVelocity;
    private Vector3 m_vPrevInputValues;

    private float m_fRotateMin = -90f;                     // Relative value in degrees
    private float m_fRotateMax = 90;                     // Relative value in degrees    
    private float m_fDeadZone = 10f;
    private Vector3 rotateAround = Vector3.right;
    private Quaternion minQuaternion;
    private Quaternion maxQuaternion;
    private float m_fRotateRange;    

    private float m_AeroFactor;
}

