// Compile gate and Editor state for a Propulsion PR review. Reports whether scripts compile,
// whether Burst compilation is on (the core's Burst test is Ignored, not passed, without it),
// the embedded yohash packages Unity has registered, and every type in the loaded
// Yohash.Propulsion assemblies, so you can check that the PR's new types resolve and its
// deleted ones don't. Run with:
//   unity command script eval --file .claude/skills/review-pr/editor-state.cs --timeout 60 --result-only
var sb = new System.Text.StringBuilder();
sb.AppendLine("scriptCompilationFailed=" + UnityEditor.EditorUtility.scriptCompilationFailed);
sb.AppendLine("isCompiling=" + UnityEditor.EditorApplication.isCompiling);
sb.AppendLine("isPlaying=" + UnityEditor.EditorApplication.isPlaying);

// Burst is optional in consumers' projects, so look it up by name rather than referencing it
var burst = System.Type.GetType("Unity.Burst.BurstCompiler, Unity.Burst");
if (burst == null)
{
  sb.AppendLine("burst=not installed");
}
else
{
  // BurstCompiler.IsEnabled follows Jobs > Burst > Enable Compilation
  sb.AppendLine("burst.IsEnabled=" + burst.GetProperty("IsEnabled").GetValue(null));
}

foreach (var p in UnityEditor.PackageManager.PackageInfo.GetAllRegisteredPackages())
  if (p.name.StartsWith("com.yohash."))
    sb.AppendLine("package " + p.name + " " + p.version + " (" + p.source + ")");

var assemblies = new System.Collections.Generic.List<System.Reflection.Assembly>();
foreach (var a in System.AppDomain.CurrentDomain.GetAssemblies())
  if (a.GetName().Name.StartsWith("yohash.", System.StringComparison.OrdinalIgnoreCase))
    assemblies.Add(a);
assemblies.Sort((x, y) => string.CompareOrdinal(x.GetName().Name, y.GetName().Name));

foreach (var a in assemblies)
{
  System.Type[] types;
  try { types = a.GetTypes(); }
  catch (System.Reflection.ReflectionTypeLoadException e) { types = e.Types; }
  var names = new System.Collections.Generic.List<string>();
  foreach (var t in types)
    if (t != null && !t.FullName.StartsWith("<") && !t.FullName.Contains("+<") && !t.FullName.StartsWith("UnitySourceGenerated"))
      names.Add(t.FullName);
  names.Sort(System.StringComparer.Ordinal);
  sb.AppendLine("assembly " + a.GetName().Name + " (" + names.Count + " types)");
  foreach (var n in names)
    sb.AppendLine("  " + n);
}
return sb.ToString();
