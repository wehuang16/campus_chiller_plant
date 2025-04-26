within campus_chiller_plant.Obsolete.Examples;
model FullPlantOriginal

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
  campus_chiller_plant.Examples.BaseClasses.TesPlant tesPlant(m_flow_nominal=
        mCHWSec_flow_nominal)
    annotation (Placement(transformation(extent={{40,-24},{62,6}})));
  replaceable
    campus_chiller_plant.Examples.BaseClasses.ElectricChillerParallelOriginal pla(
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
    annotation (Placement(transformation(extent={{-114,2},{-94,22}})));
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
    annotation (Placement(transformation(extent={{-34,54},{-54,74}})));
  Modelica.Blocks.Sources.Sine loaVar(
    amplitude=213860,
    f=1/86400,
    phase=3.1415926535898,
    offset=213860,
    startTime(displayUnit="h") = 0)     "Variable demand load"
    annotation (Placement(transformation(extent={{-160,126},{-140,146}})));
  Buildings.Applications.BaseClasses.Equipment.FlowMachine_y pumCHWSec(
    redeclare final package Medium = MediumWater,
    final per=fill(perCHWPum, numChi),
    final m_flow_nominal=mCHWSec_flow_nominal,
    final dpValve_nominal=dpCHWPumVal_nominal,
    final num=numChi) "Chilled secondary loop water pumps"
    annotation (Placement(transformation(extent={{-36,-62},{-16,-42}})));
  Buildings.Fluid.FixedResistances.Junction jonConv(
    redeclare final package Medium = MediumWater,
    final m_flow_nominal={1,-1,1},
    dp_nominal={0,0,0}) annotation (Placement(transformation(
        extent={{-11,11},{11,-11}},
        rotation=180,
        origin={59,87})));
  Buildings.Fluid.FixedResistances.Junction junDiv(
    redeclare final package Medium = MediumWater,
    final m_flow_nominal={1,-1,-1},
    dp_nominal={0,0,0}) annotation (Placement(transformation(
        extent={{12,-12},{-12,12}},
        rotation=180,
        origin={50,-54})));
  campus_chiller_plant.Examples.BaseClasses.chiller_tes_plant_controller_original
    chiller_tes_plant_controller_original
    annotation (Placement(transformation(extent={{-2,150},{18,170}})));
  Buildings.Controls.OBC.CDL.Integers.Sources.TimeTable intTimTab(
    table=[0,1; 8,2; 16,3; 21,1; 24,1],
    timeScale=3600,
    period=86400)
    annotation (Placement(transformation(extent={{-256,154},{-236,174}})));
  Buildings.Controls.OBC.CDL.Routing.RealScalarReplicator reaScaRep(nout=numChi)
    annotation (Placement(transformation(extent={{-74,76},{-54,96}})));
  Buildings.Controls.OBC.CDL.Reals.Hysteresis hys(uLow=273.15 + 15, uHigh=
        273.15 + 22)
    annotation (Placement(transformation(extent={{170,164},{190,184}})));
  Modelica.Blocks.Sources.RealExpression realExpression(y=vol.heatPort.T)
    annotation (Placement(transformation(extent={{112,150},{132,170}})));
  Buildings.Fluid.FixedResistances.PressureDrop res1(
    redeclare package Medium = MediumWater,
    m_flow_nominal=pla.numChi*mCHW_flow_nominal,
    dp_nominal(displayUnit="kPa") = 500)   "Flow resistance"
    annotation (Placement(transformation(extent={{10,-10},{-10,10}},
        rotation=0,
        origin={130,60})));
  Buildings.Controls.OBC.CDL.Reals.Hysteresis largeLoadProtection(uLow=273.15
         + 23, uHigh=273.15 + 28)
    annotation (Placement(transformation(extent={{174,206},{194,226}})));
  Buildings.Controls.OBC.CDL.Reals.Switch loaAct "actual load"
    annotation (Placement(transformation(extent={{-124,178},{-104,198}})));
  Buildings.Controls.OBC.CDL.Reals.Sources.Constant const(k=0)
    annotation (Placement(transformation(extent={{-214,188},{-194,208}})));
  Buildings.Fluid.FixedResistances.PressureDrop res2(
    redeclare package Medium = MediumWater,
    m_flow_nominal=pla.numChi*mCHW_flow_nominal,
    dp_nominal(displayUnit="kPa") = 500)   "Flow resistance"
    annotation (Placement(transformation(extent={{10,-10},{-10,10}},
        rotation=270,
        origin={68,34})));
equation
  connect(fixHeaFlo.port,vol.heatPort)
    annotation (Line(points={{-80,136},{170,136},{170,122},{176,122}},
                                                              color={191,0,0}));
  connect(res.port_b,pla. port_aSerCoo) annotation (Line(points={{-54,64},{-66,
          64},{-66,40},{-116,40},{-116,10.6667},{-114,10.6667}},
                                               color={0,127,255}));
  connect(weaDat.weaBus,pla.weaBus)
    annotation (Line(points={{-140,96},{-104,96},{-104,22}},
      color={255,204,51}));
  connect(pla.TCHWSupSet,TCHWSupSet. y) annotation (Line(points={{-114.667,
          17.3333},{-134,17.3333},{-134,-104},{-143,-104}},
                                               color={0,0,127}));
  connect(pla.port_bSerCoo, pumCHWSec.port_a) annotation (Line(points={{-94,
          10.6667},{-46,10.6667},{-46,-52},{-36,-52}},
                                                     color={0,127,255}));
  connect(junDiv.port_1, pumCHWSec.port_b)
    annotation (Line(points={{38,-54},{-10,-54},{-10,-52},{-16,-52}},
                                                        color={0,127,255}));
  connect(junDiv.port_3, tesPlant.port_a) annotation (Line(points={{50,-42},{50,
          -30},{68,-30},{68,-21.3333},{62.4632,-21.3333}},
                                                       color={0,127,255}));
  connect(junDiv.port_2, vol.ports[1]) annotation (Line(points={{62,-54},{185,
          -54},{185,112}},      color={0,127,255}));
  connect(jonConv.port_2, res.port_a) annotation (Line(points={{48,87},{-28,87},
          {-28,64},{-34,64}},        color={0,127,255}));
  connect(intTimTab.y[1], chiller_tes_plant_controller_original.systemCommand)
    annotation (Line(points={{-234,164},{-14,164},{-14,165.4},{-4,165.4}},
        color={255,127,0}));
  connect(chiller_tes_plant_controller_original.chillerOn, pla.on) annotation (
      Line(points={{20,166},{22,166},{22,112},{-112,112},{-112,19.3333},{
          -114.667,19.3333}}, color={255,0,255}));
  connect(chiller_tes_plant_controller_original.tesMode, tesPlant.chargeMode)
    annotation (Line(points={{20,156.6},{28,156.6},{28,2.66667},{38.8421,
          2.66667}}, color={255,0,255}));
  connect(chiller_tes_plant_controller_original.tesPumpSpeed, tesPlant.pump_speed)
    annotation (Line(points={{20,152.2},{20,-11},{38.8421,-11}}, color={0,0,127}));
  connect(reaScaRep.y, pumCHWSec.u) annotation (Line(points={{-52,86},{-44,86},
          {-44,-40},{-48,-40},{-48,-48},{-38,-48}},
                                                  color={0,0,127}));
  connect(realExpression.y, hys.u) annotation (Line(points={{133,160},{160,160},
          {160,174},{168,174}}, color={0,0,127}));
  connect(hys.y, chiller_tes_plant_controller_original.loadRequest) annotation (
     Line(points={{192,174},{200,174},{200,144},{-12,144},{-12,160},{-4,160}},
        color={255,0,255}));
  connect(chiller_tes_plant_controller_original.chillerPumpSpeed, reaScaRep.u)
    annotation (Line(points={{20,161.8},{-76,161.8},{-76,86}}, color={0,0,127}));
  connect(tesPlant.tesStatus, chiller_tes_plant_controller_original.tesStatus)
    annotation (Line(points={{63.1579,-8.83333},{96,-8.83333},{96,118},{-32,118},
          {-32,153},{-4,153}}, color={255,127,0}));
  connect(jonConv.port_1, res1.port_b) annotation (Line(points={{70,87},{114,87},
          {114,60},{120,60}}, color={0,127,255}));
  connect(res1.port_a, vol.ports[2])
    annotation (Line(points={{140,60},{187,60},{187,112}}, color={0,127,255}));
  connect(realExpression.y, largeLoadProtection.u) annotation (Line(points={{
          133,160},{160,160},{160,216},{172,216}}, color={0,0,127}));
  connect(largeLoadProtection.y, loaAct.u2) annotation (Line(points={{196,216},
          {204,216},{204,232},{-136,232},{-136,188},{-126,188}}, color={255,0,
          255}));
  connect(loaVar.y, loaAct.u3) annotation (Line(points={{-139,136},{-134,136},{
          -134,180},{-126,180}}, color={0,0,127}));
  connect(loaAct.y, fixHeaFlo.Q_flow) annotation (Line(points={{-102,188},{-94,
          188},{-94,152},{-110,152},{-110,136},{-100,136}}, color={0,0,127}));
  connect(const.y, loaAct.u1) annotation (Line(points={{-192,198},{-134,198},{
          -134,196},{-126,196}}, color={0,0,127}));
  connect(tesPlant.port_b, res2.port_a) annotation (Line(points={{62.3474,
          4.66667},{68,4.66667},{68,24}}, color={0,127,255}));
  connect(res2.port_b, jonConv.port_3) annotation (Line(points={{68,44},{68,70},
          {59,70},{59,76}}, color={0,127,255}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)),
    experiment(
      StopTime=172800,
      Interval=60,
      __Dymola_Algorithm="Dassl"));
end FullPlantOriginal;
