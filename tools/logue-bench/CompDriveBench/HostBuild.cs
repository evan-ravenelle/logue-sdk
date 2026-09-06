using System;
using System.Diagnostics;
using System.IO;
using System.Threading.Tasks;

namespace CompDriveBench;

/// <summary>
/// Builds a unit project for the host.
///
/// A unit's device artifact (.drmlgunit on drumlogue) is a 32-bit ARM ELF —
/// macOS cannot dlopen it. So the bench compiles the same sources a second
/// time for the host and loads that. The user picks the project; this hides
/// the second artifact.
/// </summary>
internal static class HostBuild
{
    public record Result(bool Ok, string? DylibPath, string Log);

    /// <summary>Walks up from the app directory to find test/host/unit.mk.</summary>
    public static string? FindHostDir()
    {
        var dir = new DirectoryInfo(AppContext.BaseDirectory);
        for (int i = 0; i < 10 && dir is not null; i++, dir = dir.Parent)
        {
            var candidate = Path.Combine(dir.FullName, "host", "unit.mk");
            if (File.Exists(candidate)) return Path.GetDirectoryName(candidate);
            candidate = Path.Combine(dir.FullName, "unit.mk");
            if (File.Exists(candidate)) return dir.FullName;
        }
        return null;
    }

    public static bool LooksLikeUnitProject(string dir) =>
        File.Exists(Path.Combine(dir, "config.mk")) &&
        (File.Exists(Path.Combine(dir, "unit.cc")) || File.Exists(Path.Combine(dir, "header.c")));

    public static async Task<Result> BuildAsync(string projectDir, string platform)
    {
        string? hostDir = FindHostDir();
        if (hostDir is null)
            return new Result(false, null, "Could not locate test/host/unit.mk from the app directory.");

        if (!LooksLikeUnitProject(projectDir))
            return new Result(false, null,
                $"{Path.GetFileName(projectDir)} does not look like a unit project " +
                "(expected config.mk alongside unit.cc / header.c).");

        var psi = new ProcessStartInfo("make")
        {
            WorkingDirectory = hostDir,
            RedirectStandardOutput = true,
            RedirectStandardError = true,
            UseShellExecute = false
        };
        psi.ArgumentList.Add("-f");
        psi.ArgumentList.Add("unit.mk");
        psi.ArgumentList.Add($"UNIT_DIR={projectDir}");
        psi.ArgumentList.Add($"PLATFORM={platform}");

        using var p = Process.Start(psi);
        if (p is null) return new Result(false, null, "Could not start make.");

        string stdout = await p.StandardOutput.ReadToEndAsync();
        string stderr = await p.StandardError.ReadToEndAsync();
        await p.WaitForExitAsync();

        string log = (stdout + stderr).Trim();
        if (p.ExitCode != 0) return new Result(false, null, log);

        // unit.mk prints HOSTUNIT=<path> for exactly this.
        string? path = null;
        foreach (var line in stdout.Split('\n'))
            if (line.StartsWith("HOSTUNIT=")) path = line["HOSTUNIT=".Length..].Trim();

        if (path is null || !File.Exists(path))
        {
            // Nothing to rebuild; ask the makefile where the artifact lives.
            psi.ArgumentList.Add("target-path");
            using var q = Process.Start(psi);
            if (q is not null)
            {
                string qout = await q.StandardOutput.ReadToEndAsync();
                await q.WaitForExitAsync();
                foreach (var line in qout.Split('\n'))
                    if (line.StartsWith("HOSTUNIT=")) path = line["HOSTUNIT=".Length..].Trim();
            }
        }

        return path is not null && File.Exists(path)
            ? new Result(true, path, log)
            : new Result(false, null, log + "\n(built, but the artifact path was not reported)");
    }
}
