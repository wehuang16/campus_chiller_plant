within campus_chiller_plant.Plants.BaseClasses;
model TesStatusController
  parameter Real tempDeadband(unit="K")=1
    "Deadband for temperature control";
    parameter Real tempDischargedBottom(unit="K")=273.15+17
    "fully discharged temperature for the bottom of the tank";
     parameter Real tempChargedTop(unit="K")=273.15+11
    "fully charged temperature for the top of the tank";
  Buildings.Controls.OBC.CDL.Interfaces.RealInput TesTopTemp
    annotation (Placement(transformation(extent={{-142,18},{-102,58}})));
  Buildings.Controls.OBC.CDL.Interfaces.RealInput TesBottomTemp
    annotation (Placement(transformation(extent={{-140,-82},{-100,-42}})));
  Buildings.Controls.OBC.CDL.Interfaces.IntegerOutput TesMode
    annotation (Placement(transformation(extent={{100,-20},{140,20}})));
  Buildings.Controls.OBC.CDL.Reals.Hysteresis fullyDischarged(uLow=
        tempDischargedBottom - tempDeadband, uHigh=tempDischargedBottom)
    annotation (Placement(transformation(extent={{-64,-74},{-44,-54}})));
  Buildings.Controls.OBC.CDL.Reals.Hysteresis hys2(uLow=tempChargedTop, uHigh=
        tempChargedTop + tempDeadband)
    annotation (Placement(transformation(extent={{-70,30},{-50,50}})));
  Buildings.Controls.OBC.CDL.Integers.Switch intSwi
    annotation (Placement(transformation(extent={{38,-38},{58,-18}})));
  Buildings.Controls.OBC.CDL.Integers.Sources.Constant conInt(k=-1)
    annotation (Placement(transformation(extent={{-70,-22},{-50,-2}})));
  Buildings.Controls.OBC.CDL.Integers.Sources.Constant conInt1(k=0)
    annotation (Placement(transformation(extent={{-38,2},{-18,22}})));
  Buildings.Controls.OBC.CDL.Integers.Sources.Constant conInt2(k=1)
    annotation (Placement(transformation(extent={{-52,74},{-32,94}})));
  Buildings.Controls.OBC.CDL.Integers.Switch intSwi1
    annotation (Placement(transformation(extent={{10,36},{30,56}})));
  Buildings.Controls.OBC.CDL.Logical.Not fullyCharged
    annotation (Placement(transformation(extent={{-34,30},{-14,50}})));
equation
  connect(TesTopTemp, hys2.u) annotation (Line(points={{-122,38},{-80,38},{-80,
          40},{-72,40}}, color={0,0,127}));
  connect(TesBottomTemp, fullyDischarged.u) annotation (Line(points={{-120,-62},
          {-74,-62},{-74,-64},{-66,-64}}, color={0,0,127}));
  connect(fullyDischarged.y, intSwi.u2) annotation (Line(points={{-42,-64},{26,
          -64},{26,-28},{36,-28}}, color={255,0,255}));
  connect(hys2.y, fullyCharged.u)
    annotation (Line(points={{-48,40},{-36,40}}, color={255,0,255}));
  connect(fullyCharged.y, intSwi1.u2) annotation (Line(points={{-12,40},{-2,40},
          {-2,46},{8,46}}, color={255,0,255}));
  connect(conInt.y, intSwi.u1) annotation (Line(points={{-48,-12},{26,-12},{26,
          -20},{36,-20}}, color={255,127,0}));
  connect(conInt2.y, intSwi1.u1) annotation (Line(points={{-30,84},{0,84},{0,54},
          {8,54}}, color={255,127,0}));
  connect(intSwi1.y, intSwi.u3) annotation (Line(points={{32,46},{22,46},{22,
          -36},{36,-36}}, color={255,127,0}));
  connect(conInt1.y, intSwi1.u3) annotation (Line(points={{-16,12},{0,12},{0,38},
          {8,38}}, color={255,127,0}));
  connect(intSwi.y, TesMode) annotation (Line(points={{60,-28},{94,-28},{94,0},
          {120,0}}, color={255,127,0}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)));
end TesStatusController;
