#!/usr/bin/env python
# TODO Import the TurtleBot environment when ROS is installed
# from turtlesim_enacter import TurtleSimEnacter


class Agent:
    def __init__(self, _hedonist_table):
        self.hedonist_table = _hedonist_table
        self.next_action = 0
        self.anticipated_feedback = 0

    # Return the action chosen by the agent in the context of the previous feedback
    def action(self, feedback):
        # TODO: Implement the agent's decision mechanism
        self.next_action = 0
        return self.next_action

    # Return the feedback anticipated by the agent after its last action
    def anticipation(self):
        # TODO: Implement the agent's anticipation mechanism
        self.anticipated_feedback = 0
        return self.anticipated_feedback

    # Return the satisfaction of the agent from the received feedback
    def satisfaction(self, new_feedback):
        anticipation_satisfaction = (self.anticipated_feedback == new_feedback)
        hedonist_satisfaction = self.hedonist_table[self.next_action][new_feedback]
        return anticipation_satisfaction, hedonist_satisfaction


class Environment1:
    def outcome(self, action):
        if action == 0:
            return 0
        else:
            return 1


class Environment2:
    def outcome(self, action):
        if action == 0:
            return 1
        else:
            return 0


def world(agent, environment):
    outcome = 0
    for i in range(10):
        action = agent.action(outcome)
        outcome = environment.outcome(action)
        print("Action: " + str(action) + ", Anticipation: " + str(agent.anticipation()) + ", Outcome: " + str(outcome) + ", Satisfaction: " + str(agent.satisfaction(outcome)))


# TODO Define the hedonist value of interactions (action, feedback)
hedonist_table = [[0, 1], [0, 1]]
# TODO Choose an agent
a = Agent(hedonist_table)
# TODO Choose an environment
e = Environment1()
# e = TurtlesimEnacter()

world(a, e)
