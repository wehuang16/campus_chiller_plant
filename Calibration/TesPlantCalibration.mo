within campus_chiller_plant.Calibration;
model TesPlantCalibration

    package CondensorWater =  Buildings.Media.Water;
  package ChilledWater =  Buildings.Media.Water;

    parameter Modelica.Units.SI.MassFlowRate mEva_flow_nominal=1000*2*0.06
    "Nominal mass flow rate at evaporator";
  Buildings.Fluid.Storage.Stratified tan(
    redeclare package Medium = ChilledWater,
    m_flow_nominal=mEva_flow_nominal,
    VTan=10,
    hTan=5,
    dIns=0.01,
    nSeg=16,
    T_start=278.15)
    annotation (Placement(transformation(extent={{-28,2},{-2,28}})));
  Buildings.Fluid.Sources.Boundary_pT bouSupply(
    redeclare package Medium = ChilledWater,
    use_T_in=true,
    nPorts=1)
    annotation (Placement(transformation(extent={{-150,-36},{-130,-16}})));
  Buildings.Fluid.Sources.Boundary_pT bouReturn(
    redeclare package Medium = ChilledWater,
    use_T_in=true,
    nPorts=1) annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=180,
        origin={84,82})));
  Buildings.Fluid.Sensors.TemperatureTwoPort senTemSupply(redeclare package
      Medium = ChilledWater, m_flow_nominal=mEva_flow_nominal)
    annotation (Placement(transformation(extent={{18,-60},{-10,-38}})));
  Buildings.Fluid.Movers.FlowControlled_m_flow mov(
    redeclare package Medium = ChilledWater,
    addPowerToMedium=false,
    m_flow_nominal=mEva_flow_nominal)
    annotation (Placement(transformation(extent={{-76,-70},{-108,-28}})));
  Buildings.Fluid.Sensors.TemperatureTwoPort senTemReturn(redeclare package
      Medium = ChilledWater, m_flow_nominal=mEva_flow_nominal)
    annotation (Placement(transformation(extent={{46,50},{16,82}})));
  Modelica.Blocks.Sources.CombiTimeTable dataTest(
    tableOnFile=true,
    tableName="tab1",
    fileName=ModelicaServices.ExternalReferences.loadResource(
        "modelica://campus_chiller_plant/Resources/TES_trend_updated.txt"),
    columns=2:20,
    smoothness=Modelica.Blocks.Types.Smoothness.ConstantSegments,
    extrapolation=Modelica.Blocks.Types.Extrapolation.HoldLastPoint)
    annotation (Placement(transformation(extent={{-210,42},{-190,62}})));
  Buildings.Controls.OBC.UnitConversions.From_degC from_degC
    annotation (Placement(transformation(extent={{-278,-16},{-258,4}})));
  Buildings.Controls.OBC.UnitConversions.From_degC from_degC1
    annotation (Placement(transformation(extent={{24,98},{44,118}})));
  Buildings.Controls.OBC.CDL.Reals.Sources.TimeTable daySwitchover(
    table=[0,-1; 9,1; 22,-1; 24,-1],
    smoothness=Buildings.Controls.OBC.CDL.Types.Smoothness.ConstantSegments,
    timeScale=3600)
    annotation (Placement(transformation(extent={{-178,92},{-158,112}})));
  Buildings.Controls.OBC.CDL.Reals.Multiply mul
    annotation (Placement(transformation(extent={{-104,10},{-84,30}})));
equation
  connect(from_degC.y, bouSupply.T_in) annotation (Line(points={{-256,-6},{-162,
          -6},{-162,-22},{-152,-22}}, color={0,0,127}));
  connect(from_degC1.y, bouReturn.T_in) annotation (Line(points={{46,108},{114,
          108},{114,78},{96,78}}, color={0,0,127}));
  connect(dataTest.y[17], from_degC1.u) annotation (Line(points={{-189,52},{-18,
          52},{-18,108},{22,108}}, color={0,0,127}));
  connect(dataTest.y[18], from_degC.u)
    annotation (Line(points={{-189,52},{-280,52},{-280,-6}}, color={0,0,127}));
  connect(bouReturn.ports[1], senTemReturn.port_a) annotation (Line(points={{74,
          82},{50,82},{50,66},{46,66}}, color={0,127,255}));
  connect(senTemReturn.port_b, tan.port_a)
    annotation (Line(points={{16,66},{-15,66},{-15,28}}, color={0,127,255}));
  connect(senTemSupply.port_a, tan.port_b) annotation (Line(points={{18,-49},{
          18,-50},{24,-50},{24,-6},{-15,-6},{-15,2}}, color={0,127,255}));
  connect(senTemSupply.port_b, mov.port_a)
    annotation (Line(points={{-10,-49},{-76,-49}}, color={0,127,255}));
  connect(mov.port_b, bouSupply.ports[1]) annotation (Line(points={{-108,-49},{
          -108,-50},{-124,-50},{-124,-26},{-130,-26}}, color={0,127,255}));
  connect(daySwitchover.y[1], mul.u1) annotation (Line(points={{-156,102},{-148,
          102},{-148,26},{-106,26}}, color={0,0,127}));
  connect(dataTest.y[19], mul.u2) annotation (Line(points={{-189,52},{-150,52},
          {-150,14},{-106,14}}, color={0,0,127}));
  connect(mul.y, mov.m_flow_in) annotation (Line(points={{-82,20},{-74,20},{-74,
          -14},{-92,-14},{-92,-23.8}}, color={0,0,127}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)),
    experiment(
      StartTime=7138800,
      StopTime=7894980,
      Interval=60,
      __Dymola_Algorithm="Dassl"));
end TesPlantCalibration;
