within campus_chiller_plant.Examples;
model FullPlant

            package MediumAir = Buildings.Media.Air;
  package MediumWater = Buildings.Media.Water;
    package MediumPropyleneGlycol =
      Buildings.Media.Antifreeze.PropyleneGlycolWater (property_T=273.15+50, X_a=
            0.4);
  BaseClasses.TesPlant tesPlant
    annotation (Placement(transformation(extent={{-18,-8},{20,28}})));
  Buildings.Controls.OBC.CDL.Reals.Sources.Sin sin(
    amplitude=0.25,
    freqHz=1/10000,
    offset=0.75)
    annotation (Placement(transformation(extent={{-100,-10},{-80,10}})));
  Buildings.Controls.OBC.CDL.Logical.Sources.Constant con(k=true)
    annotation (Placement(transformation(extent={{-134,32},{-114,52}})));
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
    annotation (Placement(transformation(extent={{122,-42},{142,-22}})));
  Buildings.BoundaryConditions.WeatherData.ReaderTMY3 weaDat(final
      computeWetBulbTemperature=true, filNam=
        Modelica.Utilities.Files.loadResource(
        "modelica://Buildings/Resources/weatherdata/USA_CA_San.Francisco.Intl.AP.724940_TMY3.mos"))
    "Weather data"
    annotation (Placement(transformation(extent={{72,-2},{92,18}})));
  Modelica.Blocks.Sources.BooleanConstant on
    "On signal of the plant"
    annotation (Placement(transformation(extent={{72,-42},{92,-22}})));
  Modelica.Blocks.Sources.Constant TCHWSupSet(k=TCHWSet)
    "Chilled water supply temperature setpoint"
    annotation (Placement(transformation(extent={{72,-82},{92,-62}})));
  Buildings.Fluid.MixingVolumes.MixingVolume vol(
    nPorts=2,
    redeclare package Medium = Medium,
    m_flow_nominal=pla.numChi*mCHW_flow_nominal,
    V=0.5,
    energyDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial) "Mixing volume"
    annotation (Placement(transformation(extent={{182,-2},{202,18}})));
  Modelica.Thermal.HeatTransfer.Sources.PrescribedHeatFlow fixHeaFlo(T_ref=
        293.15)
    "Fixed heat flow rate"
    annotation (Placement(transformation(extent={{132,38},{152,58}})));
  Buildings.Fluid.FixedResistances.PressureDrop res(
    redeclare package Medium = Medium,
    m_flow_nominal=pla.numChi*mCHW_flow_nominal,
    dp_nominal(displayUnit="kPa") = 60000) "Flow resistance"
    annotation (Placement(transformation(extent={{202,-72},{182,-52}})));
  Modelica.Blocks.Sources.Sine loaVar(
    amplitude=913865,
    f=1/126900,
    offset=913865,
    startTime(displayUnit="h") = 21600) "Variable demand load"
    annotation (Placement(transformation(extent={{72,38},{92,58}})));
equation
  connect(sin.y, tesPlant.pump_speed) annotation (Line(points={{-78,0},{-30,0},
          {-30,7.6},{-20,7.6}}, color={0,0,127}));
  connect(con.y, tesPlant.u) annotation (Line(points={{-112,42},{-28,42},{-28,
          24},{-20,24}}, color={255,0,255}));
  connect(fixHeaFlo.port,vol.heatPort)
    annotation (Line(points={{152,48},{164,48},{164,8},{182,8}},
                                                              color={191,0,0}));
  connect(vol.ports[1],res. port_a) annotation (Line(points={{191,-2},{212,-2},
          {212,-62},{202,-62}},   color={0,127,255}));
  connect(res.port_b,pla. port_aSerCoo) annotation (Line(points={{182,-62},{118,
          -62},{118,-33.3333},{122,-33.3333}}, color={0,127,255}));
  connect(on.y,pla.on)
    annotation (Line(points={{93,-32},{94,-32},{94,-24.6667},{121.333,-24.6667}},
      color={255,0,255}));
  connect(weaDat.weaBus,pla.weaBus)
    annotation (Line(points={{92,8},{132,8},{132,-22}},
      color={255,204,51}));
  connect(fixHeaFlo.Q_flow,loaVar. y)
    annotation (Line(points={{132,48},{93,48}},color={0,0,127}));
  connect(pla.port_bSerCoo,vol. ports[2]) annotation (Line(points={{142,
          -33.3333},{152,-33.3333},{152,-2},{193,-2}},
                                          color={0,127,255}));
  connect(pla.TCHWSupSet,TCHWSupSet. y) annotation (Line(points={{121.333,
          -26.6667},{102,-26.6667},{102,-72},{93,-72}},
                                               color={0,0,127}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)));
end FullPlant;
