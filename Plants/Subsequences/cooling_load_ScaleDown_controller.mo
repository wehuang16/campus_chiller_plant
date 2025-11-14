within campus_chiller_plant.Plants.Subsequences;
model cooling_load_ScaleDown_controller

  parameter Real lowerHysteresisTemp(unit="K")=273.15+11.11;
  parameter Real upperHysteresisTemp(unit="K")=273.15+11.67;
  Buildings.Controls.OBC.CDL.Interfaces.RealInput hx_water_temperature
    "heat exchanger water temperature"
    annotation (Placement(transformation(extent={{-140,36},{-100,76}})));
  Buildings.Controls.OBC.CDL.Reals.Line lin
    annotation (Placement(transformation(extent={{10,36},{30,56}})));
  Buildings.Controls.OBC.CDL.Reals.Sources.Constant con2(k=lowerHysteresisTemp)
    annotation (Placement(transformation(extent={{-34,80},{-14,100}})));
  Buildings.Controls.OBC.CDL.Reals.Sources.Constant con3(k=1)
    annotation (Placement(transformation(extent={{-40,44},{-20,64}})));
  Buildings.Controls.OBC.CDL.Reals.Sources.Constant con4(k=upperHysteresisTemp)
    annotation (Placement(transformation(extent={{-72,12},{-52,32}})));
  Buildings.Controls.OBC.CDL.Reals.Sources.Constant con5(k=0)
    annotation (Placement(transformation(extent={{-78,-26},{-58,-6}})));
  Buildings.Controls.OBC.CDL.Interfaces.RealOutput building_cooling_load_actual
    annotation (Placement(transformation(extent={{100,-20},{140,20}})));
  Buildings.Controls.OBC.CDL.Interfaces.RealInput building_cooling_load
    annotation (Placement(transformation(extent={{-140,-74},{-100,-34}})));
  Buildings.Controls.OBC.CDL.Reals.Multiply loaActBlock annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={50,-26})));
equation
  connect(con2.y,lin. x1) annotation (Line(points={{-12,90},{8,90},{8,54}},
                      color={0,0,127}));
  connect(con3.y,lin. f1) annotation (Line(points={{-18,54},{0,54},{0,50},{8,50}},
                                color={0,0,127}));
  connect(con4.y,lin. x2) annotation (Line(points={{-50,22},{-2,22},{-2,42},{8,
          42}},       color={0,0,127}));
  connect(con5.y,lin. f2)
    annotation (Line(points={{-56,-16},{0,-16},{0,38},{8,38}},
                                                           color={0,0,127}));
  connect(hx_water_temperature, lin.u)
    annotation (Line(points={{-120,56},{-46,56},{-46,40},{-6,40},{-6,46},{8,46}},
                                                             color={0,0,127}));
  connect(lin.y, loaActBlock.u1) annotation (Line(points={{32,46},{40,46},{40,
          -12},{38,-12},{38,-20}}, color={0,0,127}));
  connect(building_cooling_load, loaActBlock.u2) annotation (Line(points={{-120,
          -54},{30,-54},{30,-32},{38,-32}}, color={0,0,127}));
  connect(loaActBlock.y, building_cooling_load_actual) annotation (Line(points=
          {{62,-26},{96,-26},{96,0},{120,0}}, color={0,0,127}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)));
end cooling_load_ScaleDown_controller;
