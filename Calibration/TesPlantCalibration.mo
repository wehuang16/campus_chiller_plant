within campus_chiller_plant.Calibration;
model TesPlantCalibration

    package CondensorWater =  Buildings.Media.Water;
  package ChilledWater =  Buildings.Media.Water;

    parameter Modelica.Units.SI.MassFlowRate mEva_flow_nominal=65*0.9997
    "Nominal mass flow rate at evaporator";
  Buildings.Fluid.Storage.StratifiedEnhanced
                                     tan(
    redeclare package Medium = ChilledWater,
    m_flow_nominal=mEva_flow_nominal,
    VTan=2650,
    hTan=8.4352,
    dIns=0.01,
    nSeg=16,
    T_start(displayUnit="K") = 281.655)
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
  Buildings.Fluid.Sensors.TemperatureTwoPort supply_temperature_simulation(
      redeclare package Medium = ChilledWater, m_flow_nominal=mEva_flow_nominal)
    annotation (Placement(transformation(extent={{18,-60},{-10,-38}})));
  Buildings.Fluid.Movers.FlowControlled_m_flow mov(
    redeclare package Medium = ChilledWater,
    addPowerToMedium=false,
    m_flow_nominal=mEva_flow_nominal)
    annotation (Placement(transformation(extent={{-76,-70},{-108,-28}})));
  Buildings.Fluid.Sensors.TemperatureTwoPort return_temperature_simulation(
      redeclare package Medium = ChilledWater, m_flow_nominal=mEva_flow_nominal)
    annotation (Placement(transformation(extent={{46,50},{16,82}})));
  Modelica.Blocks.Sources.CombiTimeTable dataTes(
    tableOnFile=true,
    tableName="tab1",
    fileName=ModelicaServices.ExternalReferences.loadResource(
        "modelica://campus_chiller_plant/Resources/TES_trend_updated.txt"),
    columns=2:21,
    smoothness=Modelica.Blocks.Types.Smoothness.ConstantSegments,
    extrapolation=Modelica.Blocks.Types.Extrapolation.HoldLastPoint)
    annotation (Placement(transformation(extent={{-336,70},{-316,90}})));
  Buildings.Controls.OBC.UnitConversions.From_degC supply_temperature_experimental
    annotation (Placement(transformation(extent={{-278,-16},{-258,4}})));
  Buildings.Controls.OBC.UnitConversions.From_degC return_temperature_experimental
    annotation (Placement(transformation(extent={{24,98},{44,118}})));
  Modelica.Blocks.Math.Gain tes_flow(k=0.9997)
    "convert L/s to kg/s assuming 999.7kg/m3 water" annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={-152,30})));
  Buildings.Controls.OBC.UnitConversions.From_degC
                            tank_average_temperature_experimental
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={-200,-82})));
  Buildings.Controls.OBC.CDL.Reals.MultiSum mulSum(nin=16)
    annotation (Placement(transformation(extent={{104,8},{124,28}})));
  Modelica.Thermal.HeatTransfer.Sensors.TemperatureSensor tempTan[16]
    " tank tempearture" annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={48,18})));
  Modelica.Blocks.Math.Gain gai6(k=1/16) annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={166,14})));
  Buildings.Controls.OBC.CDL.Reals.Sources.Constant con7(k=0)
    annotation (Placement(transformation(extent={{-174,94},{-154,114}})));
  Buildings.HeatTransfer.Sources.PrescribedHeatFlow heat_gain_top
    annotation (Placement(transformation(extent={{-66,16},{-46,36}})));
  Buildings.HeatTransfer.Sources.PrescribedHeatFlow heat_gain_all[16]
    annotation (Placement(transformation(extent={{-60,-8},{-40,12}})));
  Buildings.Controls.OBC.CDL.Reals.Sources.Constant con8[16](k=0)
    annotation (Placement(transformation(extent={{-158,-110},{-138,-90}})));
  Buildings.Controls.OBC.UnitConversions.To_degC tank_average_temperature_simulation
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={224,-14})));
  Modelica.Blocks.Sources.CombiTimeTable dataChiller(
    tableOnFile=true,
    tableName="tab1",
    fileName=ModelicaServices.ExternalReferences.loadResource(
        "modelica://campus_chiller_plant/Resources/chiller_trend_updated.txt"),

    columns=2:11,
    smoothness=Modelica.Blocks.Types.Smoothness.ConstantSegments,
    extrapolation=Modelica.Blocks.Types.Extrapolation.HoldLastPoint)
    annotation (Placement(transformation(extent={{114,-114},{134,-94}})));
equation
  connect(supply_temperature_experimental.y, bouSupply.T_in) annotation (Line(
        points={{-256,-6},{-162,-6},{-162,-22},{-152,-22}}, color={0,0,127}));
  connect(return_temperature_experimental.y, bouReturn.T_in) annotation (Line(
        points={{46,108},{114,108},{114,78},{96,78}}, color={0,0,127}));
  connect(bouReturn.ports[1], return_temperature_simulation.port_a) annotation (
     Line(points={{74,82},{50,82},{50,66},{46,66}}, color={0,127,255}));
  connect(return_temperature_simulation.port_b, tan.port_a)
    annotation (Line(points={{16,66},{-15,66},{-15,28}}, color={0,127,255}));
  connect(supply_temperature_simulation.port_a, tan.port_b) annotation (Line(
        points={{18,-49},{18,-50},{24,-50},{24,-6},{-15,-6},{-15,2}}, color={0,
          127,255}));
  connect(supply_temperature_simulation.port_b, mov.port_a)
    annotation (Line(points={{-10,-49},{-76,-49}}, color={0,127,255}));
  connect(mov.port_b, bouSupply.ports[1]) annotation (Line(points={{-108,-49},{
          -108,-50},{-124,-50},{-124,-26},{-130,-26}}, color={0,127,255}));
  connect(dataTes.y[20], tes_flow.u) annotation (Line(points={{-315,80},{-290,
          80},{-290,30},{-164,30}}, color={0,0,127}));
  connect(tes_flow.y, mov.m_flow_in) annotation (Line(points={{-141,30},{-92,30},
          {-92,-23.8}}, color={0,0,127}));
  connect(dataTes.y[18], return_temperature_experimental.u) annotation (Line(
        points={{-315,80},{10,80},{10,108},{22,108}}, color={0,0,127}));
  connect(dataTes.y[19], supply_temperature_experimental.u) annotation (Line(
        points={{-315,80},{-290,80},{-290,-6},{-280,-6}}, color={0,0,127}));
  connect(dataTes.y[17], tank_average_temperature_experimental.u) annotation (
      Line(points={{-315,80},{-290,80},{-290,-82},{-212,-82}}, color={0,0,127}));
  connect(tan.heaPorVol, tempTan.port) annotation (Line(points={{-15,15},{-16,15},
          {-16,34},{32,34},{32,18},{38,18}}, color={191,0,0}));
  connect(tempTan.T, mulSum.u)
    annotation (Line(points={{59,18},{102,18}}, color={0,0,127}));
  connect(mulSum.y, gai6.u) annotation (Line(points={{126,18},{144,18},{144,14},
          {154,14}}, color={0,0,127}));
  connect(con7.y, heat_gain_top.Q_flow) annotation (Line(points={{-152,104},{
          -144,104},{-144,58},{-66,58},{-66,26}}, color={0,0,127}));
  connect(heat_gain_top.port, tan.heaPorVol[1]) annotation (Line(points={{-46,26},
          {-46,24},{-6,24},{-6,60},{-70,60},{-70,-32},{-15,-32},{-15,14.6344}},
                     color={191,0,0}));
  connect(heat_gain_all.port, tan.heaPorVol) annotation (Line(points={{-40,2},{
          -40,12},{-14,12},{-14,15},{-15,15}}, color={191,0,0}));
  connect(con8.y, heat_gain_all.Q_flow) annotation (Line(points={{-136,-100},{
          -66,-100},{-66,-6},{-68,-6},{-68,2},{-60,2}}, color={0,0,127}));
  connect(gai6.y,tank_average_temperature_simulation. u) annotation (Line(
        points={{177,14},{204,14},{204,-14},{212,-14}},       color={0,0,127}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)),
    experiment(
      StartTime=7171200,
      StopTime=8985600,
      Interval=60,
      __Dymola_Algorithm="Dassl"));
end TesPlantCalibration;
