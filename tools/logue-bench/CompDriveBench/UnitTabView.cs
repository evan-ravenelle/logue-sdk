using Avalonia;
using Avalonia.Controls;
using Avalonia.Layout;
using Avalonia.Media;
using System;
using System.Linq;

namespace CompDriveBench;

/// <summary>
/// One tab's worth of UI for a loaded unit: its own parameter set, plus
/// whichever side panels that unit actually supports. Everything is built
/// from what the unit reports through the normalized ABI, so a drumlogue
/// masterfx and an nts-3 genericfx produce different panels from the same
/// code path.
/// </summary>
public class UnitTabView : UserControl
{
    public int Slot { get; }

    private readonly byte[] _bmp = new byte[32];
    private int _bitmapParamId = -1;
    private BitmapMeter? _meter;
    private TextBlock? _bmpBytes;
    private TouchPad? _pad;
    private TextBlock? _touchLabel;
    private bool _building;

    public UnitTabView(int slot)
    {
        Slot = slot;

        var paramHost = new StackPanel { Spacing = 3 };
        BuildParameters(paramHost);

        var side = new StackPanel { Spacing = 8, Width = 250, Margin = new Thickness(12, 0, 0, 0) };
        BuildSidePanels(side);

        var grid = new Grid { ColumnDefinitions = new ColumnDefinitions("*,Auto") };
        var scroller = new ScrollViewer { Content = paramHost };
        Grid.SetColumn(scroller, 0);
        Grid.SetColumn(side, 1);
        grid.Children.Add(scroller);
        grid.Children.Add(side);
        Content = grid;
    }

    private static TextBlock Dim(string text, int size = 11) => new()
    {
        Text = text,
        Foreground = new SolidColorBrush(Color.FromRgb(128, 134, 139)),
        FontSize = size
    };

    private void BuildParameters(StackPanel host)
    {
        _building = true;
        int count = Native.cdh_param_count(Slot);

        host.Children.Add(Dim($"{Native.UnitPlatform(Slot)}  ·  " +
                              $"{Native.cdh_in_channels(Slot)} in  ·  {count} params"));

        for (int page = 0; page * 4 < count; page++)
        {
            var h = Dim($"PAGE {page + 1}");
            h.Margin = new Thickness(0, page == 0 ? 8 : 12, 0, 2);
            host.Children.Add(h);

            for (int slotIdx = 0; slotIdx < 4; slotIdx++)
            {
                int id = page * 4 + slotIdx;
                if (id >= count) break;
                if (Native.cdh_param_get(Slot, id, out var info) != 0) continue;

                if (string.IsNullOrEmpty(info.Name)) { host.Children.Add(Dim($"  {id}  —")); continue; }

                if (info.Type == 13) // bitmaps
                {
                    _bitmapParamId = id;
                    host.Children.Add(new TextBlock
                    {
                        Text = $"  {id}  {info.Name}  → bitmap (right)",
                        Foreground = new SolidColorBrush(Color.FromRgb(138, 180, 248)),
                        FontSize = 12
                    });
                    continue;
                }

                host.Children.Add(BuildRow(id, info));
            }
        }
        _building = false;
    }

    private Control BuildRow(int id, Native.ParamInfo info)
    {
        var row = new Grid
        {
            ColumnDefinitions = new ColumnDefinitions("28,130,*,86"),
            Margin = new Thickness(0, 1)
        };
        var idText = Dim(id.ToString());
        idText.VerticalAlignment = VerticalAlignment.Center;
        var name = new TextBlock { Text = info.Name, VerticalAlignment = VerticalAlignment.Center };

        int initial = Native.cdh_get_param(Slot, id);
        var slider = new Slider
        {
            Minimum = info.Min, Maximum = info.Max, Value = initial,
            SmallChange = 1, LargeChange = 1, TickFrequency = 1,
            IsSnapToTickEnabled = true, VerticalAlignment = VerticalAlignment.Center
        };
        var value = new TextBlock
        {
            Text = ParamFormat.Display(info, initial),
            Foreground = new SolidColorBrush(Color.FromRgb(154, 160, 166)),
            FontFamily = new FontFamily("Menlo,Consolas,monospace"),
            FontSize = 12,
            TextAlignment = TextAlignment.Right,
            VerticalAlignment = VerticalAlignment.Center
        };

        var captured = info;
        slider.PropertyChanged += (_, e) =>
        {
            if (_building || e.Property != Avalonia.Controls.Primitives.RangeBase.ValueProperty) return;
            int raw = (int)Math.Round(slider.Value);
            Native.cdh_set_param(Slot, id, raw);
            value.Text = ParamFormat.Display(captured, raw);
        };

        Grid.SetColumn(idText, 0); Grid.SetColumn(name, 1);
        Grid.SetColumn(slider, 2); Grid.SetColumn(value, 3);
        row.Children.Add(idText); row.Children.Add(name);
        row.Children.Add(slider); row.Children.Add(value);
        return row;
    }

    private void BuildSidePanels(StackPanel side)
    {
        if (_bitmapParamId >= 0)
        {
            _meter = new BitmapMeter { Width = 220, Height = 220 };
            _bmpBytes = Dim("", 10);
            _bmpBytes.FontFamily = new FontFamily("Menlo,Consolas,monospace");
            _bmpBytes.TextWrapping = TextWrapping.Wrap;
            side.Children.Add(Dim("BITMAP PARAM"));
            side.Children.Add(Dim("unit_get_param_bmp_value()", 10));
            side.Children.Add(new Border
            {
                BorderBrush = new SolidColorBrush(Color.FromRgb(60, 64, 67)),
                BorderThickness = new Thickness(1), Padding = new Thickness(4), Child = _meter
            });
            side.Children.Add(_bmpBytes);
        }

        if (Native.cdh_has_touch(Slot) != 0)
        {
            _pad = new TouchPad { Width = 220, Height = 220 };
            _touchLabel = Dim("", 10);
            _touchLabel.FontFamily = new FontFamily("Menlo,Consolas,monospace");
            _pad.Touch += (x, y, state) =>
            {
                Native.cdh_touch(Slot, 0, x, y, state);
                _touchLabel.Text = $"x {x:0.000}  y {y:0.000}  " +
                    (state == Native.TouchPressed ? "pressed" :
                     state == Native.TouchMoved ? "moved" : "released");
            };
            side.Children.Add(Dim("KAOSS PAD"));
            side.Children.Add(Dim("unit_touch_event()", 10));
            side.Children.Add(new Border
            {
                BorderBrush = new SolidColorBrush(Color.FromRgb(60, 64, 67)),
                BorderThickness = new Thickness(1), Child = _pad
            });
            side.Children.Add(_touchLabel);
        }
    }

    /// <summary>Called on the UI timer, only for the visible tab.</summary>
    public void Poll()
    {
        if (_meter is null || _bitmapParamId < 0) return;
        if (Native.cdh_get_bitmap(Slot, _bitmapParamId, _bmp) != 0) return;
        _meter.Update(_bmp);
        if (_bmpBytes is not null)
            _bmpBytes.Text = string.Join(" ", _bmp.Select(b => b.ToString("X2")));
    }
}
