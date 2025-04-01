within campus_chiller_plant.Examples.BaseClasses;
model ChillerStagingDataProcessing
  Buildings.Controls.OBC.CDL.Interfaces.BooleanInput stagingCommand[2]
    annotation (Placement(transformation(extent={{-140,38},{-100,78}})));
  Buildings.Controls.OBC.CDL.Interfaces.RealInput setpoint
    annotation (Placement(transformation(extent={{-140,-86},{-100,-46}})));
  Buildings.Controls.OBC.CDL.Interfaces.RealOutput setpointOutput[2]
    annotation (Placement(transformation(extent={{100,-18},{140,22}})));
  Buildings.Controls.OBC.CDL.Conversions.BooleanToReal booToRea[2]
    annotation (Placement(transformation(extent={{-52,34},{-32,54}})));
  Buildings.Controls.OBC.CDL.Routing.RealScalarReplicator reaScaRep(nout=2)
    annotation (Placement(transformation(extent={{-24,-62},{-4,-42}})));
  Buildings.Controls.OBC.CDL.Reals.MultiplyByParameter gai(k=0.5)
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={-60,-64})));
  Buildings.Controls.OBC.CDL.Reals.Multiply mul[2]
    annotation (Placement(transformation(extent={{24,-2},{44,18}})));
equation
  connect(stagingCommand, booToRea.u) annotation (Line(points={{-120,58},{-62,
          58},{-62,44},{-54,44}}, color={255,0,255}));
  connect(setpoint, gai.u) annotation (Line(points={{-120,-66},{-80,-66},{-80,
          -64},{-72,-64}}, color={0,0,127}));
  connect(gai.y, reaScaRep.u) annotation (Line(points={{-48,-64},{-34,-64},{-34,
          -52},{-26,-52}}, color={0,0,127}));
  connect(booToRea.y, mul.u1) annotation (Line(points={{-30,44},{14,44},{14,14},
          {22,14}}, color={0,0,127}));
  connect(reaScaRep.y, mul.u2) annotation (Line(points={{-2,-52},{14,-52},{14,2},
          {22,2}}, color={0,0,127}));
  connect(mul.y, setpointOutput)
    annotation (Line(points={{46,8},{94,8},{94,2},{120,2}}, color={0,0,127}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)));
end ChillerStagingDataProcessing;
