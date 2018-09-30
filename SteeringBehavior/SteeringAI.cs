/********************************************************************************************************************************************
 * 
 * Base class for SteeringAI.  Rather than just using a simple weighted approach each time, this way we can fine tune when 
 * and where certain steering behaviours are used.
 * 
 *******************************************************************************************************************************************/	

using System;
using UnityEngine;
using UnitySteer;
using System.Linq;
using System.Collections.Generic;

public enum SteeringAIMode { Weighted, DefaultAI }

public class SteeringAI : MonoBehaviour
{
	protected virtual void Awake ()
	{
		var allSteerings 		= GetComponents<Steering>();
        Steerings = allSteerings.Where(x => !x.IsPostProcess).ToArray();
        SteeringPostprocessors = allSteerings.Where(x => x.IsPostProcess).ToArray();
    }

	protected virtual void Start ()
	{
		if (m_nFlockID >= 0)
		{
			m_AI.MFactionAI.AddToFlock(m_AI, m_nFlockID);
		}
	}

	/********************************************************************************************************************************************
	 * 
	 * Public Functions
	 * 
	 *******************************************************************************************************************************************/

	public virtual Vector3 CalculateSteerings ()
	{
		Vector3 vResult = Vector3.zero;

		if (SteeringAIMode.Weighted == AIMode)
		{
			vResult = WeightedSteerings();
		}
		else if (SteeringAIMode.DefaultAI == AIMode)
		{
			vResult = DefaultAISteering();
		}

		return vResult;
	}

	public virtual Vector3 CalculatePostProcessSteerings (Vector3 vDesiredVelocity)
	{
		Vector3 vResult = new Vector3();

		if (SteeringAIMode.Weighted == AIMode)
		{
			vResult = WeightedPostSteerings(vDesiredVelocity);
		}
		else if (SteeringAIMode.DefaultAI == AIMode)
		{
			vResult = DefaultAIPostSteering();
		}

		return vResult;
	}

	public virtual void StopAllProcessSteering ()
	{
		for (int i = 0; i < Steerings.Length; ++i)
		{
			Steerings[i].enabled = false;
		}
	}

	public virtual void SetActiveSteering (Type steeringType, bool bActive)
	{
		bool bFound = false;

		for (int i = 0; i < Steerings.Length; ++i)
		{
			if (steeringType == Steerings[i].GetType())
			{
				if (bActive)
				{
					Steerings[i].EnableSteering();
				}
				else
				{
					Steerings[i].DisableSteering();
				}

				bFound = true;
				break;
			}
		}

		if (!bFound)
		{
			for (int i = 0; i < SteeringPostprocessors.Length; ++i)
			{
				if (steeringType == SteeringPostprocessors[i].GetType())
				{
					if (bActive)
					{
						SteeringPostprocessors[i].EnableSteering();
					}
					else
					{
						SteeringPostprocessors[i].DisableSteering();
					}

					bFound = true;
					break;
				}
			}
		}

		if (!bFound)
		{
			Debug.Log ("WARNING: Could not find " + steeringType + " in " + name);
		}
	}

	public virtual void SetActivePath (Type steeringType, AISteerPath path)
	{
		Steering pathSteer = FindSteering(steeringType);

		if (null != pathSteer)
		{
			if (typeof(SteerForPathSimplified) == steeringType)
			{
				((SteerForPathSimplified)pathSteer).Path = path.Pathway;
			}
		}
	}

	public Steering FindSteering (Type steeringType)
	{
        if (Steerings == null)
        {
            var allSteerings = GetComponents<Steering>();
            Steerings = allSteerings.Where(x => !x.IsPostProcess).ToArray();
            SteeringPostprocessors = allSteerings.Where(x => x.IsPostProcess).ToArray();
        }

		for (int i = 0; i < Steerings.Length; ++i)
		{
			if (steeringType == Steerings[i].GetType())
			{
				return Steerings[i];
			}
		}
		
		return null;
	}

	public virtual void OnDeath ()
	{
        if (Steerings != null)
        {
            for (int i = 0; i < Steerings.Length; ++i)
            {
                Steerings[i].OnDeath();
            }
        }
	}

	/// <summary>
	/// Array of steering behaviors
	/// </summary>
	public Steering[] Steerings { get; private set; }
	
	/// <summary>
	/// Array of steering post-processor behaviors
	/// </summary>
	public Steering[] SteeringPostprocessors { get; private set; }

	/// <summary>
	/// Setting for this steering AI to follow
	/// </summary>
	public SteeringAIMode AIMode
	{
		get { return m_aiMode; }
		set { m_aiMode = value; }
	}

	public List<AI> MFlock
	{
		get { return m_lFlock; }
		set { m_lFlock = value; }
	}

	public AI MAI
	{
		get 
		{
			if (m_AI == null)
			{
				m_AI = GetComponent<AI>();
			}

			return m_AI; 
		}
		set { m_AI = value; }
	}
	
	/********************************************************************************************************************************************
	 * 
	 * Protected Functions
	 * 
	 *******************************************************************************************************************************************/

	protected virtual Vector3 WeightedSteerings ()
	{
		Vector3 vForce = new Vector3();

		for(int i = 0; i < Steerings.Length; i++) 
		{
			Steering s = Steerings[i];
			if (s.enabled && !s.IsPostProcess) 
			{
				vForce += s.WeighedForce;
			}
		}

		return vForce;
	}
	
	protected virtual Vector3 WeightedPostSteerings (Vector3 vDesired)
	{        
        Vector3 vAdjustedVelocity = vDesired;

        for (int i = 0; i < SteeringPostprocessors.Length; i++) 
		{
			Steering s = SteeringPostprocessors[i];
			if (s.enabled && s.IsPostProcess) 
			{
				vAdjustedVelocity += s.WeighedForce;
			}
		}

		return vAdjustedVelocity;
	}

	protected virtual Vector3 DefaultAISteering ()
	{
		return new Vector3();
	}

	protected virtual Vector3 DefaultAIPostSteering ()
	{
		return new Vector3();
	}

	/********************************************************************************************************************************************
	 * 
	 * Public Variables
	 * 
	 ********************************************************************************************************************************************/	
	
	public SteeringAIMode 		m_aiMode				= SteeringAIMode.Weighted;
	public int 					m_nFlockID				= -1;

	/********************************************************************************************************************************************
	 * 
	 * Protected Varirables
	 * 
	 ********************************************************************************************************************************************/	
	
	protected List<AI> 			m_lFlock				= new List<AI>(3);
	protected AI				m_AI;
}
