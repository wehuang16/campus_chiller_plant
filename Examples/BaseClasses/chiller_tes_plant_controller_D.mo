within campus_chiller_plant.Examples.BaseClasses;
model chiller_tes_plant_controller_D
  Buildings.Controls.OBC.CDL.Interfaces.BooleanOutput chillerOn
    annotation (Placement(transformation(extent={{100,40},{140,80}})));
  Buildings.Controls.OBC.CDL.Interfaces.BooleanOutput tesMode
    annotation (Placement(transformation(extent={{100,-54},{140,-14}})));
  Buildings.Controls.OBC.CDL.Interfaces.RealOutput chillerPumpSpeed
    annotation (Placement(transformation(extent={{100,-2},{140,38}})));
  Buildings.Controls.OBC.CDL.Interfaces.RealOutput tesPumpSpeed
    annotation (Placement(transformation(extent={{100,-98},{140,-58}})));
  Buildings.Controls.OBC.CDL.Interfaces.IntegerInput systemCommand
    "1=baseline; 2=charge TES; 3=discharge TES"
    annotation (Placement(transformation(extent={{-140,34},{-100,74}})));
  Modelica.Blocks.Tables.CombiTable1Ds equipmentControl(table=[0,0,0.0,1,0.0; 1,
        1,1,1,1; 2,1,1,1,0; 3,0,0.0,0.0,1; 4,1,1,1,0.5],   extrapolation=
        Modelica.Blocks.Types.Extrapolation.NoExtrapolation)
    annotation (Placement(transformation(extent={{12,-20},{32,0}})));
  Buildings.Controls.OBC.CDL.Conversions.IntegerToReal intToRea
    annotation (Placement(transformation(extent={{-74,44},{-54,64}})));
  Buildings.Controls.OBC.CDL.Reals.GreaterThreshold greThr(t=0.5)
    annotation (Placement(transformation(extent={{50,50},{70,70}})));
  Buildings.Controls.OBC.CDL.Reals.GreaterThreshold greThr1(t=0.5)
    annotation (Placement(transformation(extent={{60,-42},{80,-22}})));
  Buildings.Controls.OBC.CDL.Interfaces.IntegerInput tesStatus
    "1=has capacity; 0=fully discharged"
    annotation (Placement(transformation(extent={{-140,-90},{-100,-50}})));
  Modelica.Blocks.Tables.CombiTable2Ds systemModePre(table=[0,0,1; 1,0,1; 2,2,4;
        3,0,3], extrapolation=Modelica.Blocks.Types.Extrapolation.NoExtrapolation)
    "0=do nothing; 1=chiller serves load; 2=chiller charge tes; 3= tes serves load; 4=hybrid charge; 5=hybrid serves"
    annotation (Placement(transformation(extent={{-36,46},{-16,66}})));
  Buildings.Controls.OBC.CDL.Interfaces.BooleanInput loadRequest
    "true=need cooling; false=deadband"
    annotation (Placement(transformation(extent={{-140,-20},{-100,20}})));
  Buildings.Controls.OBC.CDL.Conversions.BooleanToReal booToRea
    annotation (Placement(transformation(extent={{-80,-2},{-60,18}})));
  Buildings.Controls.OBC.CDL.Conversions.IntegerToReal intToRea1
    annotation (Placement(transformation(extent={{-78,-70},{-58,-50}})));
  Modelica.Blocks.Tables.CombiTable2Ds systemModeFinal(table=[0,-1,0,1; 0,0,0,0;
        1,1,1,1; 2,2,2,0; 3,1,3,3; 4,4,4,1], extrapolation=Modelica.Blocks.Types.Extrapolation.NoExtrapolation)
    annotation (Placement(transformation(extent={{-26,-54},{-6,-34}})));
  Buildings.Controls.OBC.CDL.Interfaces.RealOutput systemMode
    annotation (Placement(transformation(extent={{100,88},{140,128}})));
equation
  connect(equipmentControl.y[1], greThr.u) annotation (Line(points={{33,-10},{
          38,-10},{38,60},{48,60}}, color={0,0,127}));
  connect(greThr.y, chillerOn)
    annotation (Line(points={{72,60},{120,60}}, color={255,0,255}));
  connect(equipmentControl.y[2], chillerPumpSpeed) annotation (Line(points={{33,
          -10},{38,-10},{38,18},{120,18}}, color={0,0,127}));
  connect(equipmentControl.y[3], greThr1.u) annotation (Line(points={{33,-10},{
          48,-10},{48,-32},{58,-32}}, color={0,0,127}));
  connect(greThr1.y, tesMode) annotation (Line(points={{82,-32},{96,-32},{96,
          -34},{120,-34}}, color={255,0,255}));
  connect(equipmentControl.y[4], tesPumpSpeed) annotation (Line(points={{33,-10},
          {48,-10},{48,-78},{120,-78}}, color={0,0,127}));
  connect(systemCommand, intToRea.u)
    annotation (Line(points={{-120,54},{-76,54}}, color={255,127,0}));
  connect(intToRea.y, systemModePre.u1) annotation (Line(points={{-52,54},{-52,
          72},{-38,72},{-38,62}}, color={0,0,127}));
  connect(loadRequest, booToRea.u) annotation (Line(points={{-120,0},{-92,0},{
          -92,8},{-82,8}}, color={255,0,255}));
  connect(booToRea.y, systemModePre.u2)
    annotation (Line(points={{-58,8},{-38,8},{-38,50}}, color={0,0,127}));
  connect(systemModePre.y, systemModeFinal.u1) annotation (Line(points={{-15,56},
          {-12,56},{-12,-30},{-28,-30},{-28,-38}}, color={0,0,127}));
  connect(systemModeFinal.y, equipmentControl.u) annotation (Line(points={{-5,-44},
          {0,-44},{0,-10},{10,-10}}, color={0,0,127}));
  connect(systemModeFinal.y, systemMode) annotation (Line(points={{-5,-44},{-2,
          -44},{-2,12},{120,12},{120,108}}, color={0,0,127}));
  connect(tesStatus, intToRea1.u) annotation (Line(points={{-120,-70},{-88,-70},
          {-88,-60},{-80,-60}}, color={255,127,0}));
  connect(intToRea1.y, systemModeFinal.u2) annotation (Line(points={{-56,-60},{
          -36,-60},{-36,-50},{-28,-50}}, color={0,0,127}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)));
end chiller_tes_plant_controller_D;
