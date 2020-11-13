#!/usr/bin/env python
# TODO Import the Turtlesim environment when ROS is installed
# from turtlesim_enacter import TurtleSimEnacter

# Olivier Georgeon, 2020.
# This code is used to teach Develpmental AI.


class Agent:
    j = 0

    # Valeur par defaut quand il n'a pas encore fait d'action.
    latest_action = (-1, -1)

    def __init__(self, _hedonist_table):
        """ Creating our agent """
        self.hedonist_table = _hedonist_table
        self._action = 0
        self.anticipated_outcome = 0
        self.ennui = False

    def action(self, outcome):
        """ Computing the next action to enact """

        # Avant d'effectuer une nouvelle action, on enregistre l'action precedente
        # ainsi que son outcome associe.
        self.latest_action = (self._action, outcome)

        if self.ennui == True:
            if self._action == 1:
                self._action = 0
            else:
                self._action = 1
            self.ennui = False

        return self._action

    def anticipation(self):
        """ computing the anticipated outcome from the latest action """
        if self.latest_action == (-1, -1):
            # Il s'agit de sa premiere action, il n'a pas d'action precedente et d'outcome associe
            # sur lesquelles s'appuyer pour anticiper l'outcome de l'action actuelle.
            # On va donc systematiquement anticiper une autcome de 0 au depart.
            self.anticipated_outcome = 0
        else:
            # Son action est la meme que la precedente, l'outcome sera la meme.
            if self.latest_action[0] == self._action:
                self.anticipated_outcome = self.latest_action[1]
            else:
                # Son action est differente de la precedente, l'outcome sera differente.
                if self.latest_action[1] == 1:
                    self.anticipated_outcome = 0
                else:
                    self.anticipated_outcome = 1

        return self.anticipated_outcome

    def satisfaction(self, new_outcome):
        """ Computing a tuple representing the agent's satisfaction after the last interaction """

        # True if the anticipation was correct

        anticipation_satisfaction = (self.anticipated_outcome == new_outcome)
        if anticipation_satisfaction==True:
            self.j+=1
        if self.j==4:
            self.j=0
            self.ennui = True

        # The value of the enacted interaction
        hedonist_satisfaction = self.hedonist_table[self._action][new_outcome]
        return anticipation_satisfaction, hedonist_satisfaction, self.ennui


class Environment1:
    """ In Environment 1, action 0 yields outcome 0, action 1 yields outcome 1 """
    def outcome(self, action):
        if action == 0:
            return 0
        else:
            return 1


class Environment2:
    """ In Environment 2, action 0 yields outcome 1, action 1 yields outcome 0 """
    def outcome(self, action):
        if action == 0:
            return 1
        else:
            return 0


def world(agent, environment):
    """ The main loop controlling the interaction of the agent with the environment """
    outcome = 0

    for i in range(10):
        action = agent.action(outcome)
        outcome = environment.outcome(action)

        print(" Action: " + str(action) + ", Anticipation: " + str(agent.anticipation()) + ", Outcome: " + str(outcome)
              + ", Satisfaction: " + str(agent.satisfaction(outcome)))


# TODO Define the hedonist values of interactions (action, outcome)
hedonist_table = [[-1, 1], [-1, 1]]
# TODO Choose an agent
a = Agent(hedonist_table)
# TODO Choose an environment
e = Environment1()
# e = Environment2()
# e = TurtleSimEnacter()

world(a, e)