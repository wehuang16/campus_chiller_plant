within campus_chiller_plant.Plants.BaseClasses;
model FmuPatch
  import campus_chiller_plant;
  Modelica.Blocks.Sources.Sine     sine(
    amplitude=1,
    f=0.00833333333,
    offset=0.5)
    annotation (Placement(transformation(extent={{22,-22},{42,-2}})));
  Modelica.Blocks.Sources.Constant const2(k=0)
    annotation (Placement(transformation(extent={{-58,10},{-38,30}})));
  Modelica.Blocks.Sources.Constant const3(k=1)
    annotation (Placement(transformation(extent={{-54,-40},{-34,-20}})));
  campus_chiller_plant.Plants.BaseClasses.realSwitch realSwitch
    annotation (Placement(transformation(extent={{74,-38},{94,-18}})));
equation
  connect(const2.y, realSwitch.u1) annotation (Line(points={{-37,20},{-10,20},{-10,
          32},{56,32},{56,-21.8},{72,-21.8}}, color={0,0,127}));
  connect(sine.y, realSwitch.u2) annotation (Line(points={{43,-12},{2,-12},{2,-29},
          {72,-29}}, color={0,0,127}));
  connect(const3.y, realSwitch.u3) annotation (Line(points={{-33,-30},{16,-30},{
          16,-46},{71.8,-46},{71.8,-36.2}}, color={0,0,127}));
  annotation (
    Icon(coordinateSystem(preserveAspectRatio=false)),
    Diagram(coordinateSystem(preserveAspectRatio=false)),
    experiment(
      StopTime=86400,
      Interval=60,
      __Dymola_Algorithm="Cvode"));
end FmuPatch;
