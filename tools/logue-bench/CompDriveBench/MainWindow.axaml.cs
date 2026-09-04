using Avalonia.Controls;
using Avalonia.Interactivity;
using Avalonia.Platform.Storage;
using Avalonia.Threading;
using System;
using System.Text;
using System.Threading.Tasks;

namespace CompDriveBench;

public partial class MainWindow : Window
{
    private DispatcherTimer? _timer;

    public MainWindow()
    {
        InitializeComponent();

        for (int i = 0; i < Native.cdh_platform_count(); i++)
            PlatformBox.Items.Add(Native.PlatformName(i));
        PlatformBox.SelectedIndex = 0;

        OpenBtn.Click += async (_, _) => await OpenProject();
        OpenDylibBtn.Click += async (_, _) => await OpenDylib();
        CloseBtn.Click += OnCloseTab;
        LoadBtn.Click += async (_, _) => await LoadWav(false);
        LoadScBtn.Click += async (_, _) => await LoadWav(true);
        PlayBtn.Click += OnPlayStop;
        RenderBtn.Click += async (_, _) => await RenderOffline();
        LoopBox.IsCheckedChanged += (_, _) => Native.cdh_set_looping(LoopBox.IsChecked == true ? 1 : 0);
        BypassBox.IsCheckedChanged += (_, _) => Native.cdh_set_bypass(BypassBox.IsChecked == true ? 1 : 0);

        // The visible tab is the unit the audio thread renders.
        Tabs.SelectionChanged += (_, _) =>
        {
            if (Tabs.SelectedItem is TabItem { Content: UnitTabView v })
            {
                Native.cdh_set_active(v.Slot);
                ActiveLabel.Text = $"rendering: {Native.UnitName(v.Slot)} (slot {v.Slot})";
            }
        };

        _timer = new DispatcherTimer { Interval = TimeSpan.FromMilliseconds(33) };
        _timer.Tick += OnTick;
        _timer.Start();

        Closing += (_, _) => { _timer?.Stop(); Native.cdh_stop_audio(); Native.cdh_close_all(); };
        StatusLabel.Text = "Open a unit to begin.";
    }

    /// <summary>
    /// Primary flow: pick the project directory. A device artifact
    /// (.drmlgunit) is a 32-bit ARM ELF and cannot be dlopened on macOS, so
    /// the bench rebuilds the same sources for the host and loads that.
    /// </summary>
    private async Task OpenProject()
    {
        var dirs = await StorageProvider.OpenFolderPickerAsync(new FolderPickerOpenOptions
        {
            Title = "Unit project directory (contains config.mk)",
            AllowMultiple = false
        });
        if (dirs.Count == 0) return;

        string projectDir = dirs[0].Path.LocalPath;
        string platform = Native.PlatformName(PlatformBox.SelectedIndex);

        StatusLabel.Text = $"Building {System.IO.Path.GetFileName(projectDir)} for the host…";
        var result = await HostBuild.BuildAsync(projectDir, platform);
        if (!result.Ok || result.DylibPath is null)
        {
            StatusLabel.Text = "Host build failed — see below.";
            ShowLog(result.Log);
            return;
        }
        LoadDylib(result.DylibPath);
    }

    /// <summary>Escape hatch for an already-built host dylib.</summary>
    private async Task OpenDylib()
    {
        var files = await StorageProvider.OpenFilePickerAsync(new FilePickerOpenOptions
        {
            Title = "Prebuilt host dylib",
            AllowMultiple = false,
            FileTypeFilter = new[] { new FilePickerFileType("host dylib") { Patterns = new[] { "*.dylib" } } }
        });
        if (files.Count == 0) return;

        string path = files[0].Path.LocalPath;
        if (path.EndsWith(".drmlgunit", StringComparison.OrdinalIgnoreCase) ||
            path.EndsWith(".nts1mkiiunit", StringComparison.OrdinalIgnoreCase) ||
            path.EndsWith(".nts3unit", StringComparison.OrdinalIgnoreCase))
        {
            StatusLabel.Text = "That is the device artifact — a 32-bit ARM ELF, which " +
                               "cannot be loaded here. Use \u201cOpen unit project\u201d instead.";
            return;
        }
        LoadDylib(path);
    }

    private void ShowLog(string log)
    {
        string tail = log.Length > 600 ? log[^600..] : log;
        ActiveLabel.Text = tail;
    }

    private void LoadDylib(string dylibPath)
    {
        var err = new byte[256];
        int slot = Native.cdh_open(PlatformBox.SelectedIndex, dylibPath, err, err.Length);
        if (slot < 0)
        {
            int n = Array.IndexOf(err, (byte)0);
            StatusLabel.Text = "Open failed: " + Encoding.ASCII.GetString(err, 0, n < 0 ? err.Length : n);
            return;
        }

        var tab = new TabItem
        {
            Header = $"{Native.UnitName(slot)}  ·  {Native.UnitPlatform(slot)}",
            Content = new UnitTabView(slot)
        };
        Tabs.Items.Add(tab);
        Tabs.SelectedItem = tab;
        Tabs.IsVisible = true;
        EmptyHint.IsVisible = false;
        StatusLabel.Text = $"Opened “{Native.UnitName(slot)}” in slot {slot}.";
    }

    private void OnCloseTab(object? sender, RoutedEventArgs e)
    {
        if (Tabs.SelectedItem is not TabItem { Content: UnitTabView v } tab) return;
        Native.cdh_close_slot(v.Slot);
        Tabs.Items.Remove(tab);
        if (Tabs.Items.Count == 0)
        {
            Tabs.IsVisible = false;
            EmptyHint.IsVisible = true;
            ActiveLabel.Text = "";
        }
        StatusLabel.Text = $"Closed slot {v.Slot}.";
    }

    private async Task LoadWav(bool sidechain)
    {
        var files = await StorageProvider.OpenFilePickerAsync(new FilePickerOpenOptions
        {
            Title = sidechain ? "Sidechain WAV" : "Input WAV",
            AllowMultiple = false,
            FileTypeFilter = new[] { new FilePickerFileType("WAV") { Patterns = new[] { "*.wav" } } }
        });
        if (files.Count == 0) return;
        string path = files[0].Path.LocalPath;
        int rc = sidechain ? Native.cdh_load_sidechain_wav(path) : Native.cdh_load_wav(path);
        StatusLabel.Text = rc == 0
            ? $"{(sidechain ? "Sidechain" : "Input")}: {System.IO.Path.GetFileName(path)}"
            : "Load failed — 16/24/32-bit PCM or float32 only";
    }

    private void OnPlayStop(object? sender, RoutedEventArgs e)
    {
        if (Native.cdh_is_running() != 0) { Native.cdh_stop_audio(); PlayBtn.Content = "Play"; }
        else
        {
            int rc = Native.cdh_start_audio();
            if (rc != 0) { StatusLabel.Text = $"audio start failed ({rc})"; return; }
            PlayBtn.Content = "Stop";
        }
    }

    private async Task RenderOffline()
    {
        var file = await StorageProvider.SaveFilePickerAsync(new FilePickerSaveOptions
        {
            Title = "Render to WAV", SuggestedFileName = "bench_out.wav", DefaultExtension = "wav"
        });
        if (file is null) return;
        int rc = Native.cdh_render_offline(file.Path.LocalPath);
        StatusLabel.Text = rc == 0 ? $"Rendered → {file.Name}" : $"Render failed ({rc})";
    }

    private void OnTick(object? sender, EventArgs e)
    {
        Native.cdh_get_meters(out float il, out float ir, out float ol, out float or_);
        InL.SetLinear(il); InR.SetLinear(ir);
        OutL.SetLinear(ol); OutR.SetLinear(or_);

        // Output overs are a gain-staging fault the host can see directly.
        float over = Native.cdh_get_output_over();
        OutClipBar.SetFraction(over);
        OutClipVal.Text = over > 0.0005f ? $"{over * 100f:0}%" : "0%";

        // Drive saturation is internal to the unit, so it has to report it.
        // Units that expose no diagnostics simply show nothing.
        if (Tabs.SelectedItem is TabItem { Content: UnitTabView v })
        {
            v.Poll();
            if (Native.cdh_diag_count(v.Slot) > 0)
            {
                string? name = Native.DiagGet(v.Slot, 0, out float drive);
                DriveClipName.Text = name?.ToLowerInvariant() ?? "drive";
                DriveClipBar.SetFraction(drive);
                DriveClipVal.Text = drive > 0.0005f ? $"{drive * 100f:0}%" : "0%";
            }
            else
            {
                DriveClipBar.SetFraction(0f);
                DriveClipVal.Text = "—";
            }
        }

        double dur = Native.cdh_duration();
        if (dur > 0) PosLabel.Text = $"{Native.cdh_position():0.00} / {dur:0.00} s";
    }
}
