# MATLAB Spring 2014 – Research Plan

> * Group Name: Geppettos
> * Group participants names: Philipp Lütolf, Samuel Oberholzer
> * Project Title: crowd simulation

## General Introduction

(States your motivation clearly: why is it important / interesting to solve this problem?)
(Add real-world examples, if any)
(Put the problem into a historical context, from what does it originate? Are there already some proposed solutions?)

At the love parade in Duisburg 2010, 21 people died and 541 people were injured due to a crowd disaster. Big events with a lot of people bear a great risk of mass panic. One critical factor of such tragedies is the arrangement of the location. We want to model big crowds and identify the dangerous spots in a specified environment.


## The Model

(Define dependent and independent variables you want to study. Say how you want to measure them.) (Why is your model a good abtraction of the problem you want to study?) (Are you capturing all the relevant aspects of the problem?)

The model we intend to use is based on the 'social force model' from Helbing (Self-Organized Pedestrian Crowd Dynamics: Experiments, Simulations, and Design Solutions) which appears to suit our expectations. The goal is to implement and simulate the model in matlab. If the conntinuous implementation turns out to be to complicated, we will try to adapt the problem to a cellular automaton.

## Fundamental Questions

(At the end of the project you want to find the answer to these questions)
(Formulate a few, clear questions. Articulate them in sub-questions, from the more general to the more specific. )

Were are the most critical spots for people in a crowd in a given environment?
What are the possibilities to reduce the risk to individual people by rearranging the environment? 

## Expected Results

(What are the answers to the above questions that you expect to find before starting your research?)

We expect the most critical spots to be obstacles, corners, intersections and other bottlenecks where the crowd flow is disrupted.
Possible solutions to reduce the forces acting on individual people could be emergency exits and barriers.

## References 

(Add the bibliographic references you intend to use)
(Explain possible extension to the above models)
(Code / Projects Reports of the previous year)

Helbing, Buzna, Johansson, and Werner (2005): Self-Organized Pedestrian Crowd Dynamics: Experiments, Simulations, and Design Solutions
Helbing, Molnar (1995): Social force model for pedestrian dynamics


## Research Methods

(Cellular Automata, Agent-Based Model, Continuous Modeling...) (If you are not sure here: 1. Consult your colleagues, 2. ask the teachers, 3. remember that you can change it afterwards)

Agent-Based Model


## Other

(mention datasets you are going to use)

