within campus_chiller_plant.Validation;
model chiller_tes_plant_controller_validation
  CCC_test.Bakersfield_Model.chiller_tes_plant_controller
    chiller_tes_plant_controller
    annotation (Placement(transformation(extent={{-12,4},{8,24}})));
  Buildings.Controls.OBC.CDL.Integers.Sources.TimeTable intTimTab(
    table=[0,0; 6,5; 8,2; 15,4; 16,3; 21,1; 24,0],
    timeScale=3600,
    period=86400)
    annotation (Placement(transformation(extent={{-80,2},{-60,22}})));
equation
  connect(intTimTab.y[1], chiller_tes_plant_controller.systemCommand)
    annotation (Line(points={{-58,12},{-22,12},{-22,19.4},{-14,19.4}}, color={
          255,127,0}));
  annotation (
    Icon(coordinateSystem(preserveAspectRatio=false)),
    Diagram(coordinateSystem(preserveAspectRatio=false)),
    experiment(
      StopTime=172800,
      Interval=60,
      __Dymola_Algorithm="Dassl"));
end chiller_tes_plant_controller_validation;
