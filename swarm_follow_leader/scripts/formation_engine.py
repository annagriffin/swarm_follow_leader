""" Code establishing the fuzzy logic controller for staying in formation """

import fuzzylite as fl

formation_engine = fl.Engine(
    name='formation',
    description=''
)

formation_engine.input_variables = [
    fl.InputVariable(
        name='Angle',
        description='',
        enabled=True,
        minimum=-40,
        maximum=40,
        lock_range=True,
        terms=[
            fl.Ramp('very_negative', -20, -35), 
            fl.Trapezoid('negative', -25, -20, -10, -5),
            fl.Trapezoid('zero', -10, -2.5, 2.5, 10),
            fl.Trapezoid('positive', 5, 10, 20, 25),
            fl.Ramp('very_positive', 20, 35)
        ]
    ),
    fl.InputVariable(
        name='Distance',
        description='',
        enabled=True,
        minimum=-5.0,
        maximum=5.0,
        lock_range=True,
        terms=[
            # Probably need tweak these variables
            fl.Ramp('very_negative', -.25, -1), 
            fl.Trapezoid('negative', -.3, -.25, -.2, -.1),
            fl.Trapezoid('zero', -.1, -.05, .05, .1), # Probably tweak this so that the line between zero and positive is not sharp at .1
            fl.Trapezoid('positive', .1, .2, .25, .3),
            fl.Ramp('very_positive', .25, 1)
        ]
    ),                      
]

formation_engine.output_variables = [
    fl.OutputVariable(
        name='Rotation',
        description='Rotational/angular velocity',
        enabled=True,
        minimum=-1.5,
        maximum=1.5,
        lock_range=True,
        aggregation=fl.Maximum(),
        defuzzifier=fl.Centroid(),
        terms=[
            fl.Ramp('very_right', -.75, -1.5),
            fl.Triangle('right', -1.5, -.75, 0),
            fl.Triangle('straight_ahead', -.1, 0, .1),
            fl.Triangle('left', 0, .75, 1.5),
            fl.Ramp('very_left', .75, 1.5),
        ]
    ),
    fl.OutputVariable(
        name='Velocity',
        description='Translational velocity',
        enabled=True,
        minimum=-.5,
        maximum=.7,
        lock_range=True,
        aggregation=fl.Maximum(),
        defuzzifier=fl.Centroid(),
        terms=[
            fl.Ramp('reverse', 0, -.5),
            fl.Triangle('stop', -.05, 0, .05),
            fl.Triangle('slow', 0, .1, .2),
            fl.Triangle('normal', .1, .5, .6), 
            fl.Ramp('fast', .6, .7)
        ]
    ),
]

formation_engine.rule_blocks = [
    fl.RuleBlock(
        name="mamdani",
        description="",
        enabled=True,
        conjunction=fl.Minimum(),
        disjunction=fl.Maximum(),
        implication=fl.Minimum(),
        activation=fl.General(),
        rules=[
            # Angle very negative
            fl.Rule.create("if Angle is very_negative and Distance is very_negative \
                            then Rotation is very_left and Velocity is fast", formation_engine),
            fl.Rule.create("if Angle is very_negative and Distance is negative \
                            then Rotation is very_left and Velocity is normal", formation_engine),
            fl.Rule.create("if Angle is very_negative and Distance is zero \
                            then Rotation is very_left and Velocity is stop", formation_engine),
            fl.Rule.create("if Angle is very_negative and Distance is positive \
                            then Rotation is very_left and Velocity is slow", formation_engine),
            fl.Rule.create("if Angle is very_negative and Distance is very_positive \
                            then Rotation is very_left and Velocity is stop", formation_engine),                                                                                    
            # Angle negative
            fl.Rule.create("if Angle is negative and Distance is very_negative \
                            then Rotation is left and Velocity is fast", formation_engine),
            fl.Rule.create("if Angle is negative and Distance is negative \
                            then Rotation is left and Velocity is normal", formation_engine),
            fl.Rule.create("if Angle is negative and Distance is zero \
                            then Rotation is left and Velocity is stop", formation_engine),
            fl.Rule.create("if Angle is negative and Distance is positive \
                            then Rotation is left and Velocity is slow", formation_engine),
            fl.Rule.create("if Angle is negative and Distance is very_positive \
                            then Rotation is left and Velocity is stop", formation_engine),    
            # Angle zero
            fl.Rule.create("if Angle is zero and Distance is very_negative \
                            then Rotation is straight_ahead and Velocity is fast", formation_engine),
            fl.Rule.create("if Angle is zero and Distance is negative \
                            then Rotation is straight_ahead and Velocity is normal", formation_engine),
            fl.Rule.create("if Angle is zero and Distance is zero \
                            then Rotation is straight_ahead and Velocity is stop", formation_engine),
            fl.Rule.create("if Angle is zero and Distance is positive \
                            then Rotation is straight_ahead and Velocity is slow", formation_engine),
            fl.Rule.create("if Angle is zero and Distance is very_positive \
                            then Rotation is straight_ahead and Velocity is stop", formation_engine),  
            # Angle positive
            fl.Rule.create("if Angle is positive and Distance is very_negative \
                            then Rotation is right and Velocity is fast", formation_engine),
            fl.Rule.create("if Angle is positive and Distance is negative \
                            then Rotation is right and Velocity is normal", formation_engine),
            fl.Rule.create("if Angle is positive and Distance is zero \
                            then Rotation is right and Velocity is stop", formation_engine),
            fl.Rule.create("if Angle is positive and Distance is positive \
                            then Rotation is right and Velocity is slow", formation_engine),
            fl.Rule.create("if Angle is positive and Distance is very_positive \
                            then Rotation is right and Velocity is stop", formation_engine),
            # Angle very positive
            fl.Rule.create("if Angle is very_positive and Distance is very_negative \
                            then Rotation is very_right and Velocity is fast", formation_engine),
            fl.Rule.create("if Angle is very_positive and Distance is negative \
                            then Rotation is very_right and Velocity is normal", formation_engine),
            fl.Rule.create("if Angle is very_positive and Distance is zero \
                            then Rotation is very_right and Velocity is stop", formation_engine),
            fl.Rule.create("if Angle is very_positive and Distance is positive \
                            then Rotation is very_right and Velocity is slow", formation_engine),
            fl.Rule.create("if Angle is very_positive and Distance is very_positive \
                            then Rotation is very_right and Velocity is stop", formation_engine),                                                                                                                                                                                                                                                                                                                                                                                              
        ]
    )    
]

if __name__ == "__main__":
    # Testing out the engine
    angle = formation_engine.input_variable('Angle')
    distance = formation_engine.input_variable('Distance')

    rotation = formation_engine.output_variable('Rotation')
    velocity = formation_engine.output_variable('Velocity')

    angle.value = 25
    distance.value = .75

    formation_engine.process()

    print('Rotation:', rotation.value)
    print('Velocity:', velocity.value)