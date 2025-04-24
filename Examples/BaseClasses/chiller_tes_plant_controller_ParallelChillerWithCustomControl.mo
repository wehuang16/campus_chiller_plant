within campus_chiller_plant.Examples.BaseClasses;
model chiller_tes_plant_controller_ParallelChillerWithCustomControl
  Buildings.Controls.OBC.CDL.Interfaces.BooleanOutput chiller1On
    annotation (Placement(transformation(extent={{100,40},{140,80}})));
  Buildings.Controls.OBC.CDL.Interfaces.BooleanOutput secondaryPumpOn
    annotation (Placement(transformation(extent={{100,-48},{140,-8}})));
  Buildings.Controls.OBC.CDL.Interfaces.IntegerInput systemCommand
    "1=baseline; 2=charge TES; 3=discharge TES"
    annotation (Placement(transformation(extent={{-140,58},{-100,98}})));
  Modelica.Blocks.Tables.CombiTable1Ds equipmentControl(table=[0,0,0,0,0; 1,1,0,
        1,0; 2,0,1,0,1; 3,0,0,1,0],                        extrapolation=
        Modelica.Blocks.Types.Extrapolation.NoExtrapolation)
    annotation (Placement(transformation(extent={{8,-20},{28,0}})));
  Buildings.Controls.OBC.CDL.Conversions.IntegerToReal intToRea
    annotation (Placement(transformation(extent={{-74,44},{-54,64}})));
  Buildings.Controls.OBC.CDL.Reals.GreaterThreshold greThr(t=0.5)
    annotation (Placement(transformation(extent={{50,50},{70,70}})));
  Buildings.Controls.OBC.CDL.Reals.GreaterThreshold greThr2(t=0.5)
    annotation (Placement(transformation(extent={{54,-42},{74,-22}})));
  Buildings.Controls.OBC.CDL.Interfaces.IntegerInput tesStatus
    "1=has capacity; 0=fully discharged"
    annotation (Placement(transformation(extent={{-142,-44},{-102,-4}})));
  Modelica.Blocks.Tables.CombiTable2Ds systemModePre(table=[0,0,1; 1,0,1; 2,2,2;
        3,0,3], extrapolation=Modelica.Blocks.Types.Extrapolation.NoExtrapolation)
    "0=do nothing; 1=chiller serves load; 2=chiller charge tes; 3= tes serves load; 4=hybrid charge; 5=hybrid serves"
    annotation (Placement(transformation(extent={{-36,46},{-16,66}})));
  Buildings.Controls.OBC.CDL.Interfaces.BooleanInput loadRequest
    "true=need cooling; false=deadband"
    annotation (Placement(transformation(extent={{-140,18},{-100,58}})));
  Buildings.Controls.OBC.CDL.Conversions.BooleanToReal booToRea
    annotation (Placement(transformation(extent={{-80,-2},{-60,18}})));
  Buildings.Controls.OBC.CDL.Conversions.IntegerToReal intToRea1
    annotation (Placement(transformation(extent={{-78,-70},{-58,-50}})));
  Modelica.Blocks.Tables.CombiTable2Ds systemModeFinal(table=[0,-1,0,1; 0,0,0,0;
        1,1,1,1; 2,2,2,0; 3,1,3,3],          extrapolation=Modelica.Blocks.Types.Extrapolation.NoExtrapolation)
    annotation (Placement(transformation(extent={{-26,-54},{-6,-34}})));
  Buildings.Controls.OBC.CDL.Interfaces.RealOutput systemMode
    annotation (Placement(transformation(extent={{-20,-20},{20,20}},
        rotation=90,
        origin={0,120})));
  Buildings.Controls.OBC.CDL.Interfaces.BooleanOutput chiller2On
    annotation (Placement(transformation(extent={{100,-8},{140,32}})));
  Buildings.Controls.OBC.CDL.Interfaces.BooleanOutput bypassPumpOn
    annotation (Placement(transformation(extent={{100,-102},{140,-62}})));
  Buildings.Controls.OBC.CDL.Interfaces.RealOutput pumpFlowToLoad "in kg/s"
    annotation (Placement(transformation(
        extent={{-20,-20},{20,20}},
        rotation=270,
        origin={2,-120})));
  Buildings.Controls.OBC.CDL.Reals.GreaterThreshold greThr1(t=0.5)
    annotation (Placement(transformation(extent={{60,6},{80,26}})));
  Buildings.Controls.OBC.CDL.Reals.GreaterThreshold greThr3(t=0.5)
    annotation (Placement(transformation(extent={{60,-96},{80,-76}})));
  Buildings.Controls.OBC.CDL.Interfaces.RealInput zoneThermalLoad "in Watt"
    annotation (Placement(transformation(
        extent={{-20,-20},{20,20}},
        rotation=0,
        origin={-118,-78})));
equation
  connect(equipmentControl.y[1], greThr.u) annotation (Line(points={{29,-10},{
          38,-10},{38,60},{48,60}}, color={0,0,127}));
  connect(equipmentControl.y[3],greThr2. u) annotation (Line(points={{29,-10},{
          42,-10},{42,-32},{52,-32}}, color={0,0,127}));
  connect(systemCommand, intToRea.u)
    annotation (Line(points={{-120,78},{-86,78},{-86,54},{-76,54}},
                                                  color={255,127,0}));
  connect(intToRea.y, systemModePre.u1) annotation (Line(points={{-52,54},{-52,
          72},{-38,72},{-38,62}}, color={0,0,127}));
  connect(loadRequest, booToRea.u) annotation (Line(points={{-120,38},{-92,38},
          {-92,8},{-82,8}},color={255,0,255}));
  connect(booToRea.y, systemModePre.u2)
    annotation (Line(points={{-58,8},{-38,8},{-38,50}}, color={0,0,127}));
  connect(systemModePre.y, systemModeFinal.u1) annotation (Line(points={{-15,56},
          {-12,56},{-12,-30},{-28,-30},{-28,-38}}, color={0,0,127}));
  connect(systemModeFinal.y, equipmentControl.u) annotation (Line(points={{-5,-44},
          {0,-44},{0,-10},{6,-10}},  color={0,0,127}));
  connect(systemModeFinal.y, systemMode) annotation (Line(points={{-5,-44},{0,
          -44},{0,-10},{-2,-10},{-2,96},{0,96},{0,120}},
                                            color={0,0,127}));
  connect(tesStatus, intToRea1.u) annotation (Line(points={{-122,-24},{-90,-24},
          {-90,-60},{-80,-60}}, color={255,127,0}));
  connect(intToRea1.y, systemModeFinal.u2) annotation (Line(points={{-56,-60},{
          -36,-60},{-36,-50},{-28,-50}}, color={0,0,127}));
  connect(greThr.y, chiller1On)
    annotation (Line(points={{72,60},{120,60}}, color={255,0,255}));
  connect(greThr2.y, secondaryPumpOn) annotation (Line(points={{76,-32},{96,-32},
          {96,-28},{120,-28}}, color={255,0,255}));
  connect(greThr3.y, bypassPumpOn) annotation (Line(points={{82,-86},{94,-86},{
          94,-82},{120,-82}}, color={255,0,255}));
  connect(greThr1.y, chiller2On) annotation (Line(points={{82,16},{94,16},{94,
          12},{120,12}}, color={255,0,255}));
  connect(equipmentControl.y[2], greThr1.u) annotation (Line(points={{29,-10},{
          38,-10},{38,16},{58,16}}, color={0,0,127}));
  connect(equipmentControl.y[4], greThr3.u) annotation (Line(points={{29,-10},{
          42,-10},{42,-86},{58,-86}}, color={0,0,127}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)));
end chiller_tes_plant_controller_ParallelChillerWithCustomControl;
