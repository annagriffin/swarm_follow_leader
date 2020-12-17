""" Code establishing the fuzzy logic controller for staying in formation """

import fuzzylite as fl

fusion_engine = fl.Engine(
    name='fusion',
    description=''
)

fusion_engine.input_variables = [
    fl.InputVariable(
        name='Position_Measure',
        description='',
        enabled=True,
        minimum=0,
        maximum=2,
        lock_range=True,
        terms=[
            fl.Ramp('small', .2, 0), 
            fl.Trapezoid('medium', .1, .25, .3, .4),
            fl.Ramp('large', .35, 2)
        ]
    ),
    fl.InputVariable(
        name='Min_Laser',
        description='',
        enabled=True,
        minimum=0,
        maximum=5.0,
        lock_range=True,
        terms=[
            fl.Ramp('very_small', .05, 0), 
            fl.Trapezoid('small', .04, .1, .25, .35),
            fl.Trapezoid('medium', .3, 1, 2, 3.5),
            fl.Ramp('large', 3, 5)
        ]
    ),                      
]

fusion_engine.output_variables = [
    fl.OutputVariable(
        name='Formation_Weight',
        description='',
        enabled=True,
        minimum=0,
        maximum=1,
        lock_range=True,
        aggregation=fl.Maximum(),
        defuzzifier=fl.Centroid(),
        terms=[
            fl.Spike('very_small', .2, .1),
            fl.Spike('small', .4, .1),
            fl.Spike('medium', .7, .1),
            fl.Spike('large', 1, .1),
        ]
    ),
    fl.OutputVariable(
        name='Collision_Weight',
        description='',
        enabled=True,
        minimum=-0,
        maximum=1,
        lock_range=True,
        aggregation=fl.Maximum(),
        defuzzifier=fl.Centroid(),
        terms=[
            fl.Spike('small', 0, .1),
            fl.Spike('medium', .3, .1),
            fl.Spike('large', .7, .1),
        ]
    ),
]

fusion_engine.rule_blocks = [
    fl.RuleBlock(
        name="mamdani",
        description="",
        enabled=True,
        conjunction=fl.Minimum(),
        disjunction=fl.Maximum(),
        implication=fl.Minimum(),
        activation=fl.General(),
        rules=[
            # Position measure small
            fl.Rule.create("if Position_Measure is small and Min_Laser is very_small \
                            then Formation_Weight is very_small and Collision_Weight is large", fusion_engine),
            fl.Rule.create("if Position_Measure is small and Min_Laser is small \
                            then Formation_Weight is large and Collision_Weight is small", fusion_engine),
            fl.Rule.create("if Position_Measure is small and Min_Laser is medium \
                            then Formation_Weight is medium and Collision_Weight is small", fusion_engine),
            fl.Rule.create("if Position_Measure is small and Min_Laser is large \
                            then Formation_Weight is large and Collision_Weight is small", fusion_engine),                                                                             
            # Position measure medium
            fl.Rule.create("if Position_Measure is medium and Min_Laser is very_small \
                            then Formation_Weight is very_small and Collision_Weight is large", fusion_engine),
            fl.Rule.create("if Position_Measure is medium and Min_Laser is small \
                            then Formation_Weight is small and Collision_Weight is large", fusion_engine),
            fl.Rule.create("if Position_Measure is medium and Min_Laser is medium \
                            then Formation_Weight is large and Collision_Weight is medium", fusion_engine),
            fl.Rule.create("if Position_Measure is medium and Min_Laser is large \
                            then Formation_Weight is large and Collision_Weight is medium", fusion_engine),   
            # Position measure large
            fl.Rule.create("if Position_Measure is large and Min_Laser is very_small \
                            then Formation_Weight is very_small and Collision_Weight is large", fusion_engine),
            fl.Rule.create("if Position_Measure is large and Min_Laser is small \
                            then Formation_Weight is small and Collision_Weight is large", fusion_engine),
            fl.Rule.create("if Position_Measure is large and Min_Laser is medium \
                            then Formation_Weight is large and Collision_Weight is small", fusion_engine),
            fl.Rule.create("if Position_Measure is large and Min_Laser is large \
                            then Formation_Weight is large and Collision_Weight is small", fusion_engine),                                                                                                                                                                                                                                                                                                                                                                                            
        ]
    )    
]

if __name__ == "__main__":
    # Testing out the engine
    position_measure = fusion_engine.input_variable('Position_Measure')
    min_laser = fusion_engine.input_variable('Min_Laser')

    formation_weight = fusion_engine.output_variable('Formation_Weight')
    collision_weight = fusion_engine.output_variable('Collision_Weight')

    position_measure.value = 25
    min_laser.value = .75

    fusion_engine.process()

    print('formation_weight:', formation_weight.value)
    print('collision_weight:', collision_weight.value)