# README

[![PyPI version](https://badge.fury.io/py/interaction-engine.svg)](https://badge.fury.io/py/interaction-engine)
[![Build Status](https://travis-ci.com/robotpt/interaction-engine.svg?branch=master)](https://travis-ci.com/robotpt/interaction-engine)
[![Coverage Status](https://coveralls.io/repos/github/robotpt/interaction-engine/badge.svg)](https://coveralls.io/github/robotpt/interaction-engine)
[![Downloads](https://pepy.tech/badge/interaction-engine)](https://pepy.tech/project/interaction-engine)

A framework for turn-taking interactions. 

## Example

This is from `example.py`, using the terminal interface, and demonstrates several things:

* A plan-based structure that uses graphs to direct content flow
* Textual variation, including ordered variation in for the pschology questions
* Writing user input to a database
* Reading from the database to populate text
* Error checking through tests that are run on data input, including automatically 
  displaying an error message, if the user gives an invalid input
* Confirmation of valid user input, which is achieved by specifying a boolean argument 
  when creating the message
* The use of a generic interface class that is extended to make this interaction display
  on the terminal


      =====================
      Hola
       0. Hi
      >>> 0
      =====================
      What's your name?
      >>> AJ
      =====================
      'AJ', right?
       0. Yes
       1. No
      >>> 1
      =====================
      What's your name?
      >>> Audrow
      =====================
      'Audrow', right?
       0. Yes
       1. No
      >>> 0
      =====================
      Alright, Audrow, how old are you?
      >>> 1000
      =====================
      Enter a number between 0 and 200
       0. Okay
       1. Oops
      >>> 0
      =====================
      Alright, Audrow, how old are you?
      >>> 27
      =====================
      How are you?
       0. Good
       1. Okay
       2. Bad
      >>> 0
      =====================
      How do you feel about the following statement? 'I am the life of the party'
       0. Strongly agree
       1. Agree
       2. Neutral
       3. disagree
       4. Strongly disagree
      >>> 1
      =====================
      How do you feel about the following statement? 'I am always prepared'
       0. Strongly agree
       1. Agree
       2. Neutral
       3. disagree
       4. Strongly disagree
      >>> 1
      =====================
      How do you feel about the following statement? 'I get stressed out easily'
       0. Strongly agree
       1. Agree
       2. Neutral
       3. disagree
       4. Strongly disagree
      >>> 3
      =====================
      Bye
       0. Bye
       1. See ya!
      >>> 1
      =========================
      Currently in the database
      {'answers': ['Agree',
                   'Agree',
                   'disagree'],
       'question_idx': 3,
       'user_age': 27.0,
       'user_name': 'Audrow'}

 
## Setup

### Option 1: Clone the repository

> Best if you want to modify or view the code - note that you can do the following inside of a virtual environment

    git clone https://github.com/robotpt/interaction-engine
    
An easy way to setup the repository with its dependencies and with your Python path
is to use `pip`.  

    pip install -e interaction-engine

Tests can be run with the following commands.
    
    cd interaction-engine
    python3 -m unittest

### Option 2: Use Pip

> Best if you just want to use it

    python3 -m pip install interaction_engine
