within campus_chiller_plant.Examples;
model FullPlant

            package MediumAir = Buildings.Media.Air;
  package MediumWater = Buildings.Media.Water;
    package MediumPropyleneGlycol =
      Buildings.Media.Antifreeze.PropyleneGlycolWater (property_T=273.15+50, X_a=
            0.4);
parameter Integer numChi=2
    "Number of chillers";
    replaceable parameter Buildings.Fluid.Chillers.Data.ElectricEIR.ElectricEIRChiller_York_YT_1055kW_5_96COP_Vanes perChi
    "Performance data of chiller";
  parameter Modelica.Units.SI.MassFlowRate mCHW_flow_nominal=18.3
    "Nominal chilled water mass flow rate";
     parameter Modelica.Units.SI.MassFlowRate mCHWSec_flow_nominal=18.3
    "Nominal chilled water secondary loop mass flow rate";
  parameter Modelica.Units.SI.MassFlowRate mCW_flow_nominal=34.7
    "Nominal condenser water mass flow rate";
  parameter Modelica.Units.SI.PressureDifference dpCHW_nominal=44.8*1000
    "Nominal chilled water side pressure";
  parameter Modelica.Units.SI.PressureDifference dpCW_nominal=46.2*1000
    "Nominal condenser water side pressure";
  parameter Modelica.Units.SI.Power QChi_nominal=mCHW_flow_nominal*4200*(6.67
       - 18.56) "Nominal cooling capaciaty (Negative means cooling)";
  parameter Modelica.Units.SI.MassFlowRate mMin_flow=mCHW_flow_nominal*0.1
    "Minimum mass flow rate of single chiller";
  parameter Modelica.Units.SI.TemperatureDifference dTApp=3
    "Approach temperature";
  parameter Modelica.Units.SI.Power PFan_nominal=5000 "Fan power";
  // control settings
  parameter Modelica.Units.SI.Temperature TCHWSet=273.15 + 8
    "Chilled water temperature setpoint";
  parameter Modelica.Units.SI.Time tWai=30 "Waiting time";
  // pumps
  parameter Buildings.Fluid.Movers.Data.Generic perCHWPum(
    pressure=Buildings.Fluid.Movers.BaseClasses.Characteristics.flowParameters(
      V_flow=mCHW_flow_nominal/1000*{0.2,0.6,0.8,1.0},
      dp=(dpCHW_nominal+18000+30000)*{1,0.8,0.6,0.2}))
    "Performance data for chilled water pumps";
  parameter Buildings.Fluid.Movers.Data.Generic perCWPum(
    pressure=Buildings.Fluid.Movers.BaseClasses.Characteristics.flowParameters(
      V_flow=mCW_flow_nominal/1000*{0.2,0.6,1.0,1.2},
      dp=(2*dpCW_nominal+60000+6000)*{1,0.8,0.6,0.2}))
    "Performance data for condenser water pumps";
  parameter Modelica.Units.SI.Pressure dpCHWPumVal_nominal=6000
    "Nominal pressure drop of chilled water pump valve";
  parameter Modelica.Units.SI.Pressure dpCWPumVal_nominal=6000
    "Nominal pressure drop of chilled water pump valve";
  parameter Modelica.Units.SI.PressureDifference dpCooTowVal_nominal=6000
    "Nominal pressure difference of the cooling tower valve";
  BaseClasses.TesPlant tesPlant(m_flow_nominal=mCHWSec_flow_nominal)
    annotation (Placement(transformation(extent={{36,-2},{78,-40}})));
  replaceable BaseClasses.ElectricChillerParallel                  pla(
    perChi=perChi,
    dTApp=dTApp,
    perCHWPum=perCHWPum,
    perCWPum=perCWPum,
    mCHW_flow_nominal=mCHW_flow_nominal,
    dpCHW_nominal=dpCHW_nominal,
    QChi_nominal=QChi_nominal,
    mMin_flow=mMin_flow,
    mCW_flow_nominal=mCW_flow_nominal,
    dpCW_nominal=dpCW_nominal,
    TAirInWB_nominal=298.7,
    TCW_nominal=308.15,
    dT_nominal=5.56,
    TMin=288.15,
    PFan_nominal=PFan_nominal,
    dpCooTowVal_nominal=dpCooTowVal_nominal,
    dpCHWPumVal_nominal=dpCHWPumVal_nominal,
    dpCWPumVal_nominal=dpCWPumVal_nominal,
    tWai=tWai,
    energyDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial) constrainedby
    Buildings.DHC.Plants.Cooling.ElectricChillerParallel(
    perChi=perChi,
    dTApp=dTApp,
    perCHWPum=perCHWPum,
    perCWPum=perCWPum,
    mCHW_flow_nominal=mCHW_flow_nominal,
    dpCHW_nominal=dpCHW_nominal,
    QChi_nominal=QChi_nominal,
    mMin_flow=mMin_flow,
    mCW_flow_nominal=mCW_flow_nominal,
    dpCW_nominal=dpCW_nominal,
    TAirInWB_nominal=298.7,
    TCW_nominal=308.15,
    dT_nominal=5.56,
    TMin=288.15,
    PFan_nominal=PFan_nominal,
    dpCooTowVal_nominal=dpCooTowVal_nominal,
    dpCHWPumVal_nominal=dpCHWPumVal_nominal,
    dpCWPumVal_nominal=dpCWPumVal_nominal,
    tWai=tWai,
    energyDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial)
    "District cooling plant"
    annotation (Placement(transformation(extent={{-110,46},{-90,66}})));
  Buildings.BoundaryConditions.WeatherData.ReaderTMY3 weaDat(final
      computeWetBulbTemperature=true, filNam=
        Modelica.Utilities.Files.loadResource(
        "modelica://Buildings/Resources/weatherdata/USA_CA_San.Francisco.Intl.AP.724940_TMY3.mos"))
    "Weather data"
    annotation (Placement(transformation(extent={{-160,86},{-140,106}})));
  Modelica.Blocks.Sources.Constant TCHWSupSet(k=TCHWSet)
    "Chilled water supply temperature setpoint"
    annotation (Placement(transformation(extent={{-164,-114},{-144,-94}})));
  Buildings.Fluid.MixingVolumes.MixingVolume vol(
    nPorts=2,
    redeclare package Medium = MediumWater,
    m_flow_nominal=pla.numChi*mCHW_flow_nominal,
    V=20,
    energyDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial) "Mixing volume"
    annotation (Placement(transformation(extent={{176,112},{196,132}})));
  Modelica.Thermal.HeatTransfer.Sources.PrescribedHeatFlow fixHeaFlo(T_ref=
        293.15)
    "Fixed heat flow rate"
    annotation (Placement(transformation(extent={{-100,126},{-80,146}})));
  Buildings.Fluid.FixedResistances.PressureDrop res(
    redeclare package Medium = MediumWater,
    m_flow_nominal=pla.numChi*mCHW_flow_nominal,
    dp_nominal(displayUnit="kPa") = 60000) "Flow resistance"
    annotation (Placement(transformation(extent={{-34,-104},{-54,-84}})));
  Modelica.Blocks.Sources.Sine loaVar(
    amplitude=913865,
    f=1/126900,
    offset=913865,
    startTime(displayUnit="h") = 21600) "Variable demand load"
    annotation (Placement(transformation(extent={{-160,126},{-140,146}})));
  Buildings.Applications.BaseClasses.Equipment.FlowMachine_y pumCHWSec(
    redeclare final package Medium = MediumWater,
    final per=fill(perCHWPum, numChi),
    final m_flow_nominal=mCHWSec_flow_nominal,
    final dpValve_nominal=dpCHWPumVal_nominal,
    final num=numChi) "Chilled secondary loop water pumps"
    annotation (Placement(transformation(extent={{-52,56},{-32,76}})));
  Buildings.Fluid.FixedResistances.Junction jonConv(
    redeclare final package Medium = MediumWater,
    final m_flow_nominal={1,-1,1},
    dp_nominal={0,0,0}) annotation (Placement(transformation(
        extent={{12,12},{-12,-12}},
        rotation=0,
        origin={64,-98})));
  Buildings.Fluid.FixedResistances.Junction junDiv(
    redeclare final package Medium = MediumWater,
    final m_flow_nominal={1,-1,-1},
    dp_nominal={0,0,0}) annotation (Placement(transformation(
        extent={{-13,-13},{13,13}},
        rotation=0,
        origin={29,67})));
  BaseClasses.chiller_tes_plant_controller chiller_tes_plant_controller
    annotation (Placement(transformation(extent={{-2,150},{18,170}})));
  Buildings.Controls.OBC.CDL.Integers.Sources.TimeTable intTimTab(
    table=[0,1; 8,2; 16,3; 21,1; 24,1],
    timeScale=3600,
    period=86400)
    annotation (Placement(transformation(extent={{-256,154},{-236,174}})));
  Buildings.Controls.OBC.CDL.Routing.RealScalarReplicator reaScaRep(nout=numChi)
    annotation (Placement(transformation(extent={{-74,76},{-54,96}})));
  Buildings.Controls.OBC.CDL.Reals.Hysteresis hys(uLow=273.15 + 20, uHigh=
        273.15 + 30)
    annotation (Placement(transformation(extent={{170,164},{190,184}})));
  Modelica.Blocks.Sources.RealExpression realExpression(y=vol.heatPort.T)
    annotation (Placement(transformation(extent={{112,150},{132,170}})));
equation
  connect(fixHeaFlo.port,vol.heatPort)
    annotation (Line(points={{-80,136},{170,136},{170,122},{176,122}},
                                                              color={191,0,0}));
  connect(res.port_b,pla. port_aSerCoo) annotation (Line(points={{-54,-94},{
          -116,-94},{-116,54.6667},{-110,54.6667}},
                                               color={0,127,255}));
  connect(weaDat.weaBus,pla.weaBus)
    annotation (Line(points={{-140,96},{-100,96},{-100,66}},
      color={255,204,51}));
  connect(fixHeaFlo.Q_flow,loaVar. y)
    annotation (Line(points={{-100,136},{-139,136}},
                                               color={0,0,127}));
  connect(pla.TCHWSupSet,TCHWSupSet. y) annotation (Line(points={{-110.667,
          61.3333},{-134,61.3333},{-134,-104},{-143,-104}},
                                               color={0,0,127}));
  connect(pla.port_bSerCoo, pumCHWSec.port_a) annotation (Line(points={{-90,
          54.6667},{-62,54.6667},{-62,66},{-52,66}}, color={0,127,255}));
  connect(junDiv.port_1, pumCHWSec.port_b)
    annotation (Line(points={{16,67},{16,66},{-32,66}}, color={0,127,255}));
  connect(junDiv.port_3, tesPlant.port_a) annotation (Line(points={{29,54},{29,
          4},{84,4},{84,-5.37778},{78.8842,-5.37778}}, color={0,127,255}));
  connect(junDiv.port_2, vol.ports[1]) annotation (Line(points={{42,67},{170,67},
          {170,112},{185,112}}, color={0,127,255}));
  connect(jonConv.port_1, vol.ports[2]) annotation (Line(points={{76,-98},{187,
          -98},{187,112}}, color={0,127,255}));
  connect(tesPlant.port_b, jonConv.port_3) annotation (Line(points={{78.6632,
          -38.3111},{84,-38.3111},{84,-80},{64,-80},{64,-86}}, color={0,127,255}));
  connect(jonConv.port_2, res.port_a) annotation (Line(points={{52,-98},{-28,
          -98},{-28,-94},{-34,-94}}, color={0,127,255}));
  connect(intTimTab.y[1], chiller_tes_plant_controller.systemCommand)
    annotation (Line(points={{-234,164},{-14,164},{-14,165.4},{-4,165.4}},
        color={255,127,0}));
  connect(chiller_tes_plant_controller.chillerOn, pla.on) annotation (Line(
        points={{20,166},{22,166},{22,112},{-112,112},{-112,63.3333},{-110.667,
          63.3333}}, color={255,0,255}));
  connect(chiller_tes_plant_controller.tesMode, tesPlant.chargeMode)
    annotation (Line(points={{20,156.6},{26,156.6},{26,-35.7778},{33.7895,
          -35.7778}}, color={255,0,255}));
  connect(chiller_tes_plant_controller.tesPumpSpeed, tesPlant.pump_speed)
    annotation (Line(points={{20,152.2},{44,152.2},{44,-18.4667},{33.7895,
          -18.4667}}, color={0,0,127}));
  connect(reaScaRep.y, pumCHWSec.u) annotation (Line(points={{-52,86},{-44,86},
          {-44,102},{-84,102},{-84,70},{-54,70}}, color={0,0,127}));
  connect(chiller_tes_plant_controller.chillerPumpSpeed, reaScaRep.u)
    annotation (Line(points={{20,161.8},{32,161.8},{32,118},{-76,118},{-76,86}},
        color={0,0,127}));
  connect(realExpression.y, hys.u) annotation (Line(points={{133,160},{160,160},
          {160,174},{168,174}}, color={0,0,127}));
  connect(hys.y, chiller_tes_plant_controller.loadRequest) annotation (Line(
        points={{192,174},{200,174},{200,144},{-12,144},{-12,160},{-4,160}},
        color={255,0,255}));
  connect(tesPlant.tesStatus, chiller_tes_plant_controller.tesStatus)
    annotation (Line(points={{80.2105,-21.2111},{110,-21.2111},{110,130},{-4,
          130},{-4,153}}, color={255,0,255}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)));
end FullPlant;
