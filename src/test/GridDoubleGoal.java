package test;

import burlap.oomdp.core.ObjectInstance;
import burlap.oomdp.core.PropositionalFunction;
import burlap.oomdp.core.State;
import burlap.oomdp.singleagent.GroundedAction;
import burlap.oomdp.singleagent.RewardFunction;


/**
 * This class defines a reward function that returns a goal reward when any grounded form of a propositional
 * function is true in the resulting state and a default non-goal reward otherwise.
 * @author James MacGlashan
 *
 */
public class GridDoubleGoal implements RewardFunction {
	
	/**
	 * Initializes the reward function to return 1 when any grounded from of pf is true in the resulting
	 * state.
	 * @param pf the propositional function that must have a true grounded version for the goal reward to be returned.
	 */
	public GridDoubleGoal(){
	}
		
	
	@Override
	public double reward(State s, GroundedAction a, State sprime) {

		ObjectInstance goodGoal = sprime.getObjectsOfTrueClass(SoccerGridWorldDomain.CLASSLOCATION).get(0);
		ObjectInstance failGoal = sprime.getObjectsOfTrueClass(SoccerGridWorldDomain.CLASSLOCATION).get(1);
		ObjectInstance agent = sprime.getObjectsOfTrueClass(SoccerGridWorldDomain.CLASSAGENT).get(0);
		int gx = goodGoal.getDiscValForAttribute(SoccerGridWorldDomain.ATTX);
		int gy = goodGoal.getDiscValForAttribute(SoccerGridWorldDomain.ATTY);
		int fx = failGoal.getDiscValForAttribute(SoccerGridWorldDomain.ATTX);
		int fy = failGoal.getDiscValForAttribute(SoccerGridWorldDomain.ATTY);
		int ax = agent.getDiscValForAttribute(SoccerGridWorldDomain.ATTX);
		int ay = agent.getDiscValForAttribute(SoccerGridWorldDomain.ATTY);
		
		if(ax==gx && ay==gy){
			return 10;
		}
		if(ax==fx && ay==fy){
			return -30;
		}
		return -1;
		
		
	}

}
