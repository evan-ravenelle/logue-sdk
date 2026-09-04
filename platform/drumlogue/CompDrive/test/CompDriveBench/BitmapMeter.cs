using Avalonia;
using Avalonia.Controls;
using Avalonia.Media;

namespace CompDriveBench;

/// <summary>
/// Renders the unit's 16x16 1bpp parameter bitmap exactly as the SDK
/// specifies it: 32 bytes, two per row, row 0 at the top, each byte read
/// least-significant-bit first for left-to-right pixels.
/// </summary>
public class BitmapMeter : Control
{
    private byte[] _data = new byte[32];

    public void Update(byte[] data)
    {
        if (data is { Length: 32 })
        {
            _data = data;
            InvalidateVisual();
        }
    }

    public override void Render(DrawingContext ctx)
    {
        double w = Bounds.Width, h = Bounds.Height;
        if (w <= 0 || h <= 0) return;

        ctx.FillRectangle(new SolidColorBrush(Color.FromRgb(18, 20, 18)), new Rect(0, 0, w, h));

        double cw = w / 16.0, ch = h / 16.0;
        var on = new SolidColorBrush(Color.FromRgb(120, 230, 140));
        var off = new SolidColorBrush(Color.FromRgb(34, 38, 34));

        for (int row = 0; row < 16; row++)
        {
            byte lo = _data[row * 2], hi = _data[row * 2 + 1];
            for (int col = 0; col < 16; col++)
            {
                // Byte pair per row; bit 0 is the leftmost pixel of each byte.
                bool lit = col < 8 ? ((lo >> col) & 1) != 0 : ((hi >> (col - 8)) & 1) != 0;
                ctx.FillRectangle(lit ? on : off,
                    new Rect(col * cw + 0.5, row * ch + 0.5, cw - 1, ch - 1));
            }
        }
    }
}
