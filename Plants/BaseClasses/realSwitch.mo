within campus_chiller_plant.Plants.BaseClasses;
model realSwitch
  Modelica.Blocks.Interfaces.RealInput u1
    annotation (Placement(transformation(extent={{-140,42},{-100,82}})));
  Modelica.Blocks.Interfaces.RealOutput y
    annotation (Placement(transformation(extent={{100,-8},{120,12}})));

  Modelica.Blocks.Interfaces.RealInput u2
    annotation (Placement(transformation(extent={{-140,-30},{-100,10}})));
  Modelica.Blocks.Interfaces.RealInput u3
    annotation (Placement(transformation(extent={{-142,-102},{-102,-62}})));

equation
  y = if u2>=0.5 then u1 else u3;
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)));
end realSwitch;
