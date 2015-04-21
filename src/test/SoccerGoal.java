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
public class SoccerGoal implements RewardFunction {
	double							goalReward = 200;
	double							missReward = -55;
	double width = 8;
	
	
	/**
	 * Initializes the reward function to return 1 when any grounded from of pf is true in the resulting
	 * state.
	 * @param pf the propositional function that must have a true grounded version for the goal reward to be returned.
	 */
	public SoccerGoal(double width){
		this.width = width;
	}
		
	
	@Override
	public double reward(State s, GroundedAction a, State sprime) {
		ObjectInstance passAgent = sprime.getObjectsOfTrueClass(SoccerGridWorldDomain.CLASSLOCATION).get(0);
		ObjectInstance agent = sprime.getObjectsOfTrueClass(SoccerGridWorldDomain.CLASSAGENT).get(0);
		int px = passAgent.getDiscValForAttribute(SoccerGridWorldDomain.ATTX);
		int py = passAgent.getDiscValForAttribute(SoccerGridWorldDomain.ATTY);
		int ax = agent.getDiscValForAttribute(SoccerGridWorldDomain.ATTX);
		int ay = agent.getDiscValForAttribute(SoccerGridWorldDomain.ATTY);
		
		if(px==0 && py== 0 && ax==0 && ay==0){
			return missReward;
		}
		if(px==1 && py== 1 && ax==1 && ay==1){
			return goalReward;
		}
		return -30.0*ax*ax/(width*width) - 2;
		
		
	}

}
