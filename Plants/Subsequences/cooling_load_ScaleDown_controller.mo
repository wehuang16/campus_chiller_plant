within campus_chiller_plant.Plants.Subsequences;
model cooling_load_ScaleDown_controller

  parameter Real lowerHysteresisTemp(unit="K")=273.15+11.11;
  parameter Real upperHysteresisTemp(unit="K")=273.15+11.67;
  Buildings.Controls.OBC.CDL.Interfaces.RealInput hx_water_temperature
    "heat exchanger water temperature"
    annotation (Placement(transformation(extent={{-140,-20},{-100,20}})));
  Buildings.Controls.OBC.CDL.Reals.Line lin
    annotation (Placement(transformation(extent={{62,-6},{82,14}})));
  Buildings.Controls.OBC.CDL.Reals.Sources.Constant con2(k=lowerHysteresisTemp)
    annotation (Placement(transformation(extent={{-70,60},{-50,80}})));
  Buildings.Controls.OBC.CDL.Reals.Sources.Constant con3(k=1)
    annotation (Placement(transformation(extent={{-72,18},{-52,38}})));
  Buildings.Controls.OBC.CDL.Reals.Sources.Constant con4(k=upperHysteresisTemp)
    annotation (Placement(transformation(extent={{-42,-38},{-22,-18}})));
  Buildings.Controls.OBC.CDL.Reals.Sources.Constant con5(k=0)
    annotation (Placement(transformation(extent={{-48,-76},{-28,-56}})));
  Buildings.Controls.OBC.CDL.Interfaces.RealOutput cooling_load_scaling_factor
    annotation (Placement(transformation(extent={{100,-20},{140,20}})));
equation
  connect(con2.y,lin. x1) annotation (Line(points={{-48,70},{52,70},{52,12},{60,
          12}},       color={0,0,127}));
  connect(con3.y,lin. f1) annotation (Line(points={{-50,28},{50,28},{50,8},{60,
          8}},                  color={0,0,127}));
  connect(con4.y,lin. x2) annotation (Line(points={{-20,-28},{50,-28},{50,0},{
          60,0}},     color={0,0,127}));
  connect(con5.y,lin. f2)
    annotation (Line(points={{-26,-66},{52,-66},{52,-4},{60,-4}},
                                                           color={0,0,127}));
  connect(hx_water_temperature, lin.u)
    annotation (Line(points={{-120,0},{46,0},{46,4},{60,4}}, color={0,0,127}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)));
end cooling_load_ScaleDown_controller;
