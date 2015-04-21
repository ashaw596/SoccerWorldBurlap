package test;
import java.awt.Color;
import java.util.List;

import burlap.oomdp.singleagent.common.VisualActionObserver;
import burlap.behavior.singleagent.*;
import burlap.behavior.singleagent.auxiliary.StateReachability;
import burlap.behavior.singleagent.auxiliary.performance.LearningAlgorithmExperimenter;
import burlap.behavior.singleagent.auxiliary.performance.PerformanceMetric;
import burlap.behavior.singleagent.auxiliary.performance.TrialMode;
import burlap.behavior.singleagent.auxiliary.valuefunctionvis.ValueFunctionVisualizerGUI;
import burlap.behavior.singleagent.auxiliary.valuefunctionvis.common.ArrowActionGlyph;
import burlap.behavior.singleagent.auxiliary.valuefunctionvis.common.LandmarkColorBlendInterpolation;
import burlap.behavior.singleagent.auxiliary.valuefunctionvis.common.PolicyGlyphPainter2D;
import burlap.behavior.singleagent.auxiliary.valuefunctionvis.common.PolicyGlyphPainter2D.PolicyGlyphRenderStyle;
import burlap.behavior.singleagent.auxiliary.valuefunctionvis.common.StateValuePainter2D;
import burlap.behavior.singleagent.learning.GoalBasedRF;
import burlap.behavior.singleagent.learning.LearningAgent;
import burlap.behavior.singleagent.learning.LearningAgentFactory;
import burlap.behavior.singleagent.learning.tdmethods.QLearning;
import burlap.behavior.singleagent.planning.OOMDPPlanner;
import burlap.behavior.singleagent.planning.QComputablePlanner;
import burlap.behavior.singleagent.planning.StateConditionTest;
import burlap.behavior.singleagent.planning.commonpolicies.GreedyQPolicy;
import burlap.behavior.singleagent.planning.deterministic.TFGoalCondition;
import burlap.behavior.singleagent.planning.stochastic.policyiteration.PolicyIteration;
import burlap.behavior.singleagent.planning.stochastic.valueiteration.ValueIteration;
import burlap.domain.singleagent.gridworld.*;
import burlap.oomdp.auxiliary.StateGenerator;
import burlap.oomdp.auxiliary.StateParser;
import burlap.oomdp.auxiliary.common.ConstantStateGenerator;
import burlap.oomdp.auxiliary.common.UniversalStateParser;
import burlap.oomdp.core.*;
import burlap.oomdp.singleagent.*;
import burlap.oomdp.singleagent.common.*;
import burlap.oomdp.singleagent.explorer.VisualExplorer;
import burlap.oomdp.visualizer.Visualizer;
import burlap.behavior.statehashing.DiscreteStateHashFactory;

public class SoccerBehavior {

	SoccerGridWorldDomain gwdg;
	Domain domain;
	StateParser sp;
	RewardFunction rf;
	TerminalFunction tf;
	StateConditionTest goalCondition;
	State initialState;
	DiscreteStateHashFactory hashingFactory;
	
	public void visualize(String outputPath){
		Visualizer v = GridWorldVisualizer.getVisualizer(gwdg.getMap());
		EpisodeSequenceVisualizer evis = new EpisodeSequenceVisualizer(v, domain, sp, outputPath);
	}
	
	public SoccerBehavior(){
		
		//create the domain
		gwdg = new SoccerGridWorldDomain(8, 3);
		//gwdg.setMapToFourRooms(); 
		domain = gwdg.generateDomain();
		
		//create the state parser
		sp = new UniversalStateParser(domain); 
		
		//define the task
		rf = new SoccerGoal(gwdg.getWidth()); 
		tf = new SinglePFTF(domain.getPropFunction(SoccerGridWorldDomain.PFATLOCATION)); 
		goalCondition = new TFGoalCondition(tf);
		
		//set up the initial state of the task
		initialState = SoccerGridWorldDomain.getOneAgentOneLocationState(domain);
		SoccerGridWorldDomain.setAgent(initialState, 0, 0);
		SoccerGridWorldDomain.setLocation(initialState, 0, 3, 2);
		
		//set up the state hashing system
		hashingFactory = new DiscreteStateHashFactory();
		
		//Visualizer v = GridWorldVisualizer.getVisualizer(gwdg.getMap());
		//VisualExplorer exp = new VisualExplorer(domain, v, initialState);
		
		//use w-s-a-d-x
		
		//hashingFactory.setAttributesForClass(SoccerGridWorldDomain.CLASSAGENT, 
		//domain.getObjectClass(SoccerGridWorldDomain.CLASSAGENT).attributeList); 
		/*
		VisualActionObserver observer = new VisualActionObserver(domain, 
								GridWorldVisualizer.getVisualizer(gwdg.getMap()));
					((SADomain)this.domain).setActionObserverForAllAction(observer);
					observer.initGUI();			
			*/						
		
	}
	
	public void valueFunctionVisualize(QComputablePlanner planner, Policy p){
		List <State> allStates = StateReachability.getReachableStates(initialState, 
			(SADomain)domain, hashingFactory);
		LandmarkColorBlendInterpolation rb = new LandmarkColorBlendInterpolation();
		rb.addNextLandMark(0., Color.RED);
		rb.addNextLandMark(1., Color.BLUE);
		
		StateValuePainter2D svp = new StateValuePainter2D(rb);
		svp.setXYAttByObjectClass(SoccerGridWorldDomain.CLASSAGENT, SoccerGridWorldDomain.ATTX, 
			SoccerGridWorldDomain.CLASSAGENT, SoccerGridWorldDomain.ATTY);
		
		PolicyGlyphPainter2D spp = new PolicyGlyphPainter2D();
		spp.setXYAttByObjectClass(SoccerGridWorldDomain.CLASSAGENT, SoccerGridWorldDomain.ATTX, 
			SoccerGridWorldDomain.CLASSAGENT, SoccerGridWorldDomain.ATTY);
		spp.setActionNameGlyphPainter(SoccerGridWorldDomain.ACTIONNORTH, new ArrowActionGlyph(0));
		spp.setActionNameGlyphPainter(SoccerGridWorldDomain.ACTIONSOUTH, new ArrowActionGlyph(1));
		spp.setActionNameGlyphPainter(SoccerGridWorldDomain.ACTIONEAST, new ArrowActionGlyph(2));
		spp.setActionNameGlyphPainter(SoccerGridWorldDomain.ACTIONWEST, new ArrowActionGlyph(3));
		spp.setRenderStyle(PolicyGlyphRenderStyle.DISTSCALED);
		
		ValueFunctionVisualizerGUI gui = new ValueFunctionVisualizerGUI(allStates, svp, planner);
		gui.setSpp(spp);
		gui.setPolicy(p);
		gui.setBgColor(Color.GRAY);
		gui.initGUI();
	}
	
	public void ValueIterationExample(String outputPath){
		
		if(!outputPath.endsWith("/")){
			outputPath = outputPath + "/";
		}
		
		
		OOMDPPlanner planner = new ValueIteration(domain, rf, tf, 0.99, hashingFactory, 0.001, 100);
		((ValueIteration)planner).sp = sp;
		planner.planFromState(initialState);
		
		//create a Q-greedy policy from the planner
		Policy p = new GreedyQPolicy((QComputablePlanner)planner);
		
		//record the plan results to a file
		p.evaluateBehavior(initialState, rf, tf).writeToFile(outputPath + "ValueIterationResult", sp);

		this.valueFunctionVisualize((QComputablePlanner)planner, p);
	}
	
	public void PolicyIterationExample(String outputPath){
		
		if(!outputPath.endsWith("/")){
			outputPath = outputPath + "/";
		}
		
		
		OOMDPPlanner planner = new PolicyIteration(domain, rf, tf, 0.99, hashingFactory, 0.001, 10,100);
		((PolicyIteration)planner).sp = sp;
		planner.planFromState(initialState);
		
		//create a Q-greedy policy from the planner
		Policy p = new GreedyQPolicy((QComputablePlanner)planner);
		
		//record the plan results to a file
		EpisodeAnalysis ea = p.evaluateBehavior(initialState, rf, tf);
		List<Double> list = ea.rewardSequence;
		double rewards=0;
		for (Double d: list) {
			rewards += d;
			System.out.println(d);
		}
		System.out.println(rewards);
		
		ea.writeToFile(outputPath + "PolicyIterationResult", sp);
		

		this.valueFunctionVisualize((QComputablePlanner)planner, p);
	}
	
	
			
	public void QLearningExample(String outputPath){
		
		if(!outputPath.endsWith("/")){
			outputPath = outputPath + "/";
		}
		
		//creating the learning algorithm object; discount= 0.99; initialQ=0.0; learning rate=0.9
		LearningAgent agent = new QLearning(domain, rf, tf, 0.99, hashingFactory, 0., 0.9);
		

		Policy p = new GreedyQPolicy((QComputablePlanner)agent);
		
		//run learning for 100 episodes
		for(int i = 0; i < 200; i++){
			EpisodeAnalysis ea = agent.runLearningEpisodeFrom(initialState);
			ea.writeToFile(String.format("%se%03d", outputPath, i), sp); 
			System.out.println(i + ": " + ea.numTimeSteps());
		}

		this.valueFunctionVisualize((QComputablePlanner)agent, p);
		p.evaluateBehavior(initialState, rf, tf).writeToFile(outputPath + "QPlanResult", sp);
		
	}
	
	private class QLearningFactory implements LearningAgentFactory{
		public double learningRate;
		QLearningFactory(double learningRate) {
			this.learningRate = learningRate;
		}
		@Override
		public String getAgentName() {
			return "Q-learning";
		}
		
		@Override
		public LearningAgent generateAgent() {
			return new QLearning(domain, rf, tf, 0.99, hashingFactory, 0,  learningRate);
		}
	}
	
	public void experimenterAndPlotter(){
		
		//custom reward function for more interesting results
		//final RewardFunction rf = new GoalBasedRF(this.goalCondition, 5., -0.1);

		/**
		 * Create factories for Q-learning agent and SARSA agent to compare
		 */

		for (double learningRate=0.1; learningRate<=0.9; learningRate+=0.1) {
			LearningAgentFactory qLearningFactory = new QLearningFactory(learningRate);
	
			
	
			StateGenerator sg = new ConstantStateGenerator(this.initialState);
	
			LearningAlgorithmExperimenter exp = new LearningAlgorithmExperimenter((SADomain)this.domain, 
				rf, sg, 1, 6000, sp, learningRate, tf, qLearningFactory);
	
			exp.setUpPlottingConfiguration(500, 250, 2, 1000, 
				TrialMode.MOSTRECENTANDAVERAGE, 
				PerformanceMetric.CUMULATIVESTEPSPEREPISODE, 
				PerformanceMetric.AVERAGEEPISODEREWARD,
				PerformanceMetric.CUMULTAIVEREWARDPEREPISODE);
	
			exp.startExperiment();
	
			exp.writeStepAndEpisodeDataToCSV("expData");
		}


	}
	

				
	public static void main(String[] args) {


		SoccerBehavior example = new SoccerBehavior();
		String outputPath = "output/"; //directory to record results
		
		//we will call planning and learning algorithms here
		
		
		//run the visualizer
		example.ValueIterationExample(outputPath);
		example.PolicyIterationExample(outputPath);
		example.QLearningExample(outputPath);
		//example.experimenterAndPlotter();
		example.visualize(outputPath);

	}
			

}