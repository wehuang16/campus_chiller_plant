within campus_chiller_plant.Calibration;
model FullPlantCalibration
  Examples.Chiller_Storage_CoolingTower_ParallelChillerWithCustomControl2
    chiller_Storage_CoolingTower_ParallelChillerWithCustomControl2_1
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
    columns=2:9,
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
        origin={-14,-52})));
equation
  connect(dataChiller.y[5], chiller_total_flow_experimental.u) annotation (Line(
        points={{35,-16},{62,-16},{62,22},{72,22}}, color={0,0,127}));
  connect(dataChiller.y[7], chiller_1_flow_experimental.u)
    annotation (Line(points={{35,-16},{104,-16}}, color={0,0,127}));
  connect(dataChiller.y[8], chiller_2_flow_experimental.u) annotation (Line(
        points={{35,-16},{56,-16},{56,-62},{66,-62}}, color={0,0,127}));
  connect(dataTes.y[20], tes_flow_experimental.u) annotation (Line(points={{-55,
          -18},{-36,-18},{-36,-52},{-26,-52}}, color={0,0,127}));
  annotation (
    Icon(coordinateSystem(preserveAspectRatio=false)),
    Diagram(coordinateSystem(preserveAspectRatio=false)),
    experiment(
      StartTime=7171200,
      StopTime=8985600,
      Interval=60,
      __Dymola_Algorithm="Dassl"));
end FullPlantCalibration;
