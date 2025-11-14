within campus_chiller_plant.Plants.Subsequences;
model plant_mode_controller
  parameter Modelica.Units.SI.HeatFlowRate load_nominal=3000000
    "Nominal cooling rate for the load";

  Buildings.Controls.OBC.CDL.Interfaces.IntegerInput systemCommand
    "1=baseline; 2=charge TES; 3=discharge TES"
    annotation (Placement(transformation(extent={{-140,58},{-100,98}})));
  Buildings.Controls.OBC.CDL.Conversions.IntegerToReal intToRea
    annotation (Placement(transformation(extent={{-74,44},{-54,64}})));
  Buildings.Controls.OBC.CDL.Interfaces.IntegerInput tesStatus
    "1=has capacity; 0=fully discharged"
    annotation (Placement(transformation(extent={{-142,-44},{-102,-4}})));
  Modelica.Blocks.Tables.CombiTable2Ds systemModePre(table=[-999,0,1; 0,0,0; 1,
        0,1; 2,2,2; 3,0,3],
                extrapolation=Modelica.Blocks.Types.Extrapolation.NoExtrapolation)
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
        1,1,1,1; 2,2,2,2; 3,1,3,3],          extrapolation=Modelica.Blocks.Types.Extrapolation.NoExtrapolation)
    annotation (Placement(transformation(extent={{-24,-54},{-4,-34}})));
  Buildings.Controls.OBC.CDL.Interfaces.IntegerOutput
                                                   systemMode
    annotation (Placement(transformation(extent={{-20,-20},{20,20}},
        rotation=0,
        origin={120,0})));
  Buildings.Controls.OBC.CDL.Conversions.RealToInteger reaToInt annotation (
      Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={44,0})));
equation
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
          {-12,56},{-12,-30},{-26,-30},{-26,-38}}, color={0,0,127}));
  connect(tesStatus, intToRea1.u) annotation (Line(points={{-122,-24},{-90,-24},
          {-90,-60},{-80,-60}}, color={255,127,0}));
  connect(intToRea1.y, systemModeFinal.u2) annotation (Line(points={{-56,-60},{
          -36,-60},{-36,-50},{-26,-50}}, color={0,0,127}));
  connect(systemMode, reaToInt.y) annotation (Line(points={{120,0},{60,0},{60,
          22},{44,22},{44,12}},
                        color={255,127,0}));
  connect(systemModeFinal.y, reaToInt.u) annotation (Line(points={{-3,-44},{44,
          -44},{44,-12}},                                 color={0,0,127}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)));
end plant_mode_controller;
