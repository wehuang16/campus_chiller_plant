within campus_chiller_plant.Examples.Validation;
model full_chiller_plant_fmu_validation
  campus_chiller_plant_fmu chiller_plant_fmu
    annotation (Placement(transformation(extent={{-14,8},{64,36}})));
  Buildings.Controls.OBC.CDL.Reals.Sources.Sin      sin(
    amplitude=2,
    freqHz=1/86400,
    phase=4.1887902047864,
    offset=273.15 + 21)
    annotation (Placement(transformation(extent={{-92,30},{-72,50}})));
  Modelica.Blocks.Sources.Sine loaVar(
    amplitude=500000,
    f=1/86400,
    phase=4.1887902047864,
    offset=500000,
    startTime(displayUnit="h") = 0)     "Variable demand load"
    annotation (Placement(transformation(extent={{-90,-42},{-70,-22}})));
  Buildings.Controls.OBC.CDL.Integers.Sources.TimeTable custom_schedule(
    table=[0,1; 8,2; 16,3; 21,1; 24,1],
    timeScale=3600,
    period(displayUnit="s") = 86400)
    annotation (Placement(transformation(extent={{-60,44},{-40,64}})));
equation
  connect(sin.y, chiller_plant_fmu.outdoor_air_temperature) annotation (Line(
        points={{-70,40},{-36,40},{-36,42.6},{-27.4,42.6}}, color={0,0,127}));
  connect(loaVar.y, chiller_plant_fmu.building_cooling_load) annotation (Line(
        points={{-69,-32},{-30,-32},{-30,26},{-25.4,26},{-25.4,34.8}}, color={0,
          0,127}));
  connect(custom_schedule.y[1], chiller_plant_fmu.systemCommand) annotation (
      Line(points={{-38,54},{-38,62},{-16,62},{-16,48.4},{-26.6,48.4}}, color={
          255,127,0}));
  annotation (
    Icon(coordinateSystem(preserveAspectRatio=false)),
    Diagram(coordinateSystem(preserveAspectRatio=false)),
    experiment(
      StopTime=259200,
      Interval=60,
      __Dymola_Algorithm="Dassl"));
end full_chiller_plant_fmu_validation;
