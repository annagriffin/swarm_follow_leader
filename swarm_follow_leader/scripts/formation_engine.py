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
            fl.Trapazoid('negative', -25, -20, -5, -.5),
            fl.Trapazoid('zero', -5, -2.5, 2.5, 5),
            fl.Trapazoid('positive', .5, 5, 20, 25),
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
            fl.Ramp('very_negative', -.3, -1), 
            fl.Trapazoid('negative', -.5, -.3, -.25, -.1),
            fl.Trapazoid('zero', -.1, -.05, .05, .1),
            fl.Trapazoid('positive', .1, .25, .3, .5),
            fl.Ramp('very_positive', .3, 1)
        ]
    ),                      
]

formation_engine.output_variables = [
    fl.OutputVariable(
        name='Rotation',
        description='Rotational/angular velocity',
        enabled=True,
        minimum=-3,
        maximum=3,
        lock_range=True,
        aggregation=fl.Maximum(),
        defuzzifier=fl.Centroid(),
        terms=[
            fl.Ramp('very_right', -1, -3)
            fl.Triangle('right', -3, -1, 0),
            fl.Triangle('straight_ahead', -1, 0, 1),
            fl.Triangle('left', 0, 1, 3),
            fl.Ramp('very_left', 1, 3),
        ]
    ),
    fl.OutputVariable(
        name='Velocity',
        description='Translational velocity',
        enabled=True,
        minimum=-.5,
        maximum=1,
        lock_range=True,
        aggregation=fl.Maximum(),
        defuzzifier=fl.Centroid(),
        terms=[
            fl.Ramp('reverse', 0, -.5),
            fl.Triangle('slow', -.05, .3, .6),
            fl.Triangle('normal', .3, .6, 1), 
            fl.Ramp('fast', .6, 1)
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
            # Left laser near section
            fl.Rule.create("if Left_Laser is near and Right_Laser is near and Front_Laser is near \
                            then Rotation is very_right and Velocity is reverse", formation_engine),
            fl.Rule.create("if Left_Laser is near and Right_Laser is near and Front_Laser is medium \
                            then Rotation is straight_ahead and Velocity is normal", formation_engine),
            fl.Rule.create("if Left_Laser is near and Right_Laser is near and Front_Laser is far \
                            then Rotation is straight_ahead and Velocity is fast", formation_engine),
            fl.Rule.create("if Left_Laser is near and Right_Laser is medium and Front_Laser is near \
                            then Rotation is very_right and Velocity is reverse", formation_engine),
            fl.Rule.create("if Left_Laser is near and Right_Laser is medium and Front_Laser is medium \
                            then Rotation is right and Velocity is slow", formation_engine),
            fl.Rule.create("if Left_Laser is near and Right_Laser is medium and Front_Laser is far \
                            then Rotation is very_right and Velocity is normal", formation_engine),
            fl.Rule.create("if Left_Laser is near and Right_Laser is far and Front_Laser is near \
                            then Rotation is very_right and Velocity is reverse", formation_engine),
            fl.Rule.create("if Left_Laser is near and Right_Laser is far and Front_Laser is medium \
                            then Rotation is very_right and Velocity is slow", formation_engine),
            fl.Rule.create("if Left_Laser is near and Right_Laser is far and Front_Laser is far \
                            then Rotation is very_right and Velocity is normal", formation_engine),
            # Left laser medium
            fl.Rule.create("if Left_Laser is medium and Right_Laser is near and Front_Laser is near \
                            then Rotation is very_left and Velocity is reverse", formation_engine),
            fl.Rule.create("if Left_Laser is medium and Right_Laser is near and Front_Laser is medium \
                            then Rotation is very_left and Velocity is slow", formation_engine),
            fl.Rule.create("if Left_Laser is medium and Right_Laser is near and Front_Laser is far \
                            then Rotation is very_left and Velocity is slow", formation_engine),
            fl.Rule.create("if Left_Laser is medium and Right_Laser is medium and Front_Laser is near \
                            then Rotation is very_right and Velocity is reverse", formation_engine),
            fl.Rule.create("if Left_Laser is medium and Right_Laser is medium and Front_Laser is medium \
                            then Rotation is right and Velocity is normal", formation_engine),
            fl.Rule.create("if Left_Laser is medium and Right_Laser is medium and Front_Laser is far \
                            then Rotation is very_right and Velocity is normal", formation_engine),
            fl.Rule.create("if Left_Laser is medium and Right_Laser is far and Front_Laser is near \
                            then Rotation is very_right and Velocity is reverse", formation_engine),
            fl.Rule.create("if Left_Laser is medium and Right_Laser is far and Front_Laser is medium \
                            then Rotation is very_right and Velocity is slow", formation_engine),
            fl.Rule.create("if Left_Laser is medium and Right_Laser is far and Front_Laser is far \
                            then Rotation is very_right and Velocity is normal", formation_engine),
            # Left laser far
            fl.Rule.create("if Left_Laser is far and Right_Laser is near and Front_Laser is near \
                            then Rotation is very_left and Velocity is reverse", formation_engine),
            fl.Rule.create("if Left_Laser is far and Right_Laser is near and Front_Laser is medium \
                            then Rotation is very_left and Velocity is slow", formation_engine),
            fl.Rule.create("if Left_Laser is far and Right_Laser is near and Front_Laser is far \
                            then Rotation is very_left and Velocity is normal", formation_engine),
            fl.Rule.create("if Left_Laser is far and Right_Laser is medium and Front_Laser is near \
                            then Rotation is very_left and Velocity is reverse", formation_engine),
            fl.Rule.create("if Left_Laser is far and Right_Laser is medium and Front_Laser is medium \
                            then Rotation is left and Velocity is normal", formation_engine),
            fl.Rule.create("if Left_Laser is far and Right_Laser is medium and Front_Laser is far \
                            then Rotation is left and Velocity is normal", formation_engine),
            fl.Rule.create("if Left_Laser is far and Right_Laser is far and Front_Laser is near \
                            then Rotation is very_right and Velocity is reverse", formation_engine),
            fl.Rule.create("if Left_Laser is far and Right_Laser is far and Front_Laser is medium \
                            then Rotation is right and Velocity is normal", formation_engine),
            fl.Rule.create("if Left_Laser is far and Right_Laser is far and Front_Laser is far \
                            then Rotation is straight_ahead and Velocity is normal", formation_engine),                                                                                                                                                                                                                                                                                                                                                                                             
        ]
    )    
]

if __name__ == "__main__":
    # Testing out the engine
    angle = formation_engine.input_variable('Angle')
    distance = formation_engine.input_variable('Distance')

    rotation = formation_engine.output_variable('Rotation')
    velocity = formation_engine.output_variable('Velocity')

    angle.value = 2.5
    distance.value = 2.5

    formation_engine.process()

    print('Rotation:', rotation.value)
    print('Velocity:', velocity.value)