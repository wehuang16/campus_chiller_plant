within campus_chiller_plant.Calibration;
model FullPlantCalibration
  Plants.campus_chiller_plant campus_chiller_plant_fmu_v2_1
    annotation (Placement(transformation(extent={{-36,36},{42,64}})));
  Modelica.Blocks.Sources.CombiTimeTable dataTes(
    tableOnFile=true,
    tableName="tab1",
    fileName=ModelicaServices.ExternalReferences.loadResource("modelica://campus_chiller_plant/Resources/TES_trend_updated.txt"),
    columns=2:21,
    smoothness=Modelica.Blocks.Types.Smoothness.ConstantSegments,
    extrapolation=Modelica.Blocks.Types.Extrapolation.HoldLastPoint)
    annotation (Placement(transformation(extent={{-76,-28},{-56,-8}})));

  Modelica.Blocks.Sources.CombiTimeTable dataChiller(
    tableOnFile=true,
    tableName="tab1",
    fileName=ModelicaServices.ExternalReferences.loadResource("modelica://campus_chiller_plant/Resources/chiller_trend_updated.txt"),
    columns=2:11,
    smoothness=Modelica.Blocks.Types.Smoothness.ConstantSegments,
    extrapolation=Modelica.Blocks.Types.Extrapolation.HoldLastPoint)
    annotation (Placement(transformation(extent={{14,-26},{34,-6}})));

  Buildings.Controls.OBC.CDL.Reals.MultiplyByParameter chiller_1_flow_experimental(k=0.9997)
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={116,-16})));
  Buildings.Controls.OBC.CDL.Reals.MultiplyByParameter chiller_2_flow_experimental(k=0.9997)
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={78,-62})));
  Buildings.Controls.OBC.CDL.Reals.MultiplyByParameter chiller_total_flow_experimental(k=0.9997)
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={84,22})));
  Buildings.Controls.OBC.CDL.Reals.MultiplyByParameter tes_flow_experimental(k=0.9997)
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={-12,-52})));
  Buildings.Controls.OBC.UnitConversions.From_degC
                            tank_average_temperature_experimental
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={-52,-80})));
  Buildings.Controls.OBC.UnitConversions.From_degC from_degC annotation (
      Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={-84,42})));
  Buildings.Controls.OBC.CDL.Conversions.RealToInteger reaToInt
    annotation (Placement(transformation(extent={{-122,74},{-102,94}})));
equation
  connect(dataChiller.y[5], chiller_total_flow_experimental.u) annotation (Line(
        points={{35,-16},{62,-16},{62,22},{72,22}}, color={0,0,127}));
  connect(dataChiller.y[7], chiller_1_flow_experimental.u)
    annotation (Line(points={{35,-16},{104,-16}}, color={0,0,127}));
  connect(dataChiller.y[8], chiller_2_flow_experimental.u) annotation (Line(
        points={{35,-16},{56,-16},{56,-62},{66,-62}}, color={0,0,127}));
  connect(dataTes.y[20], tes_flow_experimental.u) annotation (Line(points={{-55,-18},
          {-36,-18},{-36,-52},{-24,-52}},      color={0,0,127}));
  connect(dataTes.y[17], tank_average_temperature_experimental.u) annotation (
      Line(points={{-55,-18},{-36,-18},{-36,-66},{-74,-66},{-74,-80},{-64,-80}},
        color={0,0,127}));
  connect(from_degC.y, campus_chiller_plant_fmu_v2_1.outdoor_air_temperature)
    annotation (Line(points={{-72,42},{-60,42},{-60,70.6},{-49.4,70.6}}, color=
          {0,0,127}));
  connect(dataChiller.y[9], from_degC.u) annotation (Line(points={{35,-16},{40,
          -16},{40,28},{-106,28},{-106,42},{-96,42}}, color={0,0,127}));
  connect(reaToInt.y, campus_chiller_plant_fmu_v2_1.systemCommand) annotation (
      Line(points={{-100,84},{-48.6,84},{-48.6,76.4}}, color={255,127,0}));
  connect(reaToInt.u, dataChiller.y[10]) annotation (Line(points={{-124,84},{
          -132,84},{-132,28},{46,28},{46,-16},{35,-16}}, color={0,0,127}));
  connect(campus_chiller_plant_fmu_v2_1.building_cooling_load, dataChiller.y[6])
    annotation (Line(points={{-47.4,62.8},{-47.4,28},{40,28},{40,-16},{35,-16}},
        color={0,0,127}));
  annotation (
    Icon(coordinateSystem(preserveAspectRatio=false)),
    Diagram(coordinateSystem(preserveAspectRatio=false)),
    experiment(
      StartTime=7171200,
      StopTime=8985600,
      Interval=60,
      __Dymola_Algorithm="Dassl"));
end FullPlantCalibration;
