within campus_chiller_plant.Validation;
model TesPlantValidation

            package MediumAir = Buildings.Media.Air;
  package MediumWater = Buildings.Media.Water;
    package MediumPropyleneGlycol =
      Buildings.Media.Antifreeze.PropyleneGlycolWater (property_T=273.15+50, X_a=
            0.4);
  Plants.Subsequences.TesPlant tesPlant
    annotation (Placement(transformation(extent={{-28,-18},{10,18}})));
  Buildings.Fluid.Sources.Boundary_pT bou(redeclare package Medium =
        MediumWater, nPorts=1)
    annotation (Placement(transformation(extent={{-10,-10},{10,10}},
        rotation=180,
        origin={82,36})));
  Buildings.Fluid.Sources.Boundary_pT bou1(
    redeclare package Medium = MediumWater,
    T=308.15,
    nPorts=1)
    annotation (Placement(transformation(extent={{-10,-10},{10,10}},
        rotation=180,
        origin={78,-34})));
  Buildings.Controls.OBC.CDL.Reals.Sources.Sin sin(
    amplitude=0.25,
    freqHz=1/10000,
    offset=0.75)
    annotation (Placement(transformation(extent={{-110,-20},{-90,0}})));
  Buildings.Controls.OBC.CDL.Logical.Sources.Pulse booPul(period=21600)
    annotation (Placement(transformation(extent={{-144,22},{-124,42}})));
equation
  connect(bou1.ports[1], tesPlant.port_a) annotation (Line(points={{68,-34},{16,
          -34},{16,-14.8},{10.8,-14.8}}, color={0,127,255}));
  connect(tesPlant.port_b, bou.ports[1]) annotation (Line(points={{10.6,16.4},{
          66,16.4},{66,36},{72,36}}, color={0,127,255}));
  connect(sin.y, tesPlant.pump_speed) annotation (Line(points={{-88,-10},{-40,
          -10},{-40,-2.4},{-30,-2.4}}, color={0,0,127}));
  connect(booPul.y, tesPlant.chargeMode) annotation (Line(points={{-122,32},{
          -38,32},{-38,14},{-30,14}}, color={255,0,255}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)),
    experiment(
      StopTime=86400,
      Interval=60,
      __Dymola_Algorithm="Dassl"));
end TesPlantValidation;
