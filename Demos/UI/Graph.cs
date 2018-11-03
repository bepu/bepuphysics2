using BepuUtilities;
using BepuUtilities.Collections;
using BepuUtilities.Memory;
using DemoRenderer.UI;
using DemoUtilities;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Numerics;
using System.Text;

namespace Demos.UI
{
    public interface IDataSeries
    {
        int Start { get; }
        int End { get; }
        double this[int index] { get; }
    }

    public struct GraphDescription
    {
        /// <summary>
        /// Minimum location of the graph body in pixels, not including the interval labels.
        /// </summary>
        public Vector2 BodyMinimum;
        /// <summary>
        /// Span of the graph body in pixels, not including interval labels.
        /// </summary>
        public Vector2 BodySpan;
        public Vector3 BodyLineColor;
        public float AxisLabelHeight;
        public float AxisLineRadius;
        public string HorizontalAxisLabel;
        public string VerticalAxisLabel;
        public int VerticalIntervalLabelRounding;
        public float VerticalIntervalValueScale;
        public float BackgroundLineRadius;
        public float IntervalTextHeight;
        public float IntervalTickRadius;
        /// <summary>
        /// The length of a tick mark line, measured from the axis.
        /// </summary>
        public float IntervalTickLength;
        /// <summary>
        /// Number of interval ticks along the horizontal axis, not including the start and end ticks.
        /// </summary>
        public int TargetHorizontalTickCount;
        /// <summary>
        /// Number of interval ticks along the vertical axis, not including the min and max ticks.
        /// </summary>
        public int TargetVerticalTickCount;
        public float HorizontalTickTextPadding;
        public float VerticalTickTextPadding;


        /// <summary>
        /// Minimum location of the legend in pixels.
        /// </summary>
        public Vector2 LegendMinimum;
        public float LegendNameHeight;
        public float LegendLineLength;

        public Vector3 TextColor;
        public Font Font;

        public float LineSpacingMultiplier;
        public bool ForceVerticalAxisMinimumToZero;
    }

    public class Graph
    {
        public struct Series
        {
            //The use of a string here blocks the use of unmanaged storage. Not a big deal; drawing a Graph isn't exactly performance critical.
            public string Name;
            public Vector3 LineColor;
            public float LineRadius;
            public IDataSeries Data;
        }
        List<Series> graphSeries;

        GraphDescription description;

        public ref GraphDescription Description
        {
            get
            {
                return ref description;
            }
        }

        public Graph(GraphDescription description, int initialSeriesCapacity = 8)
        {
            Description = description;
            if (initialSeriesCapacity <= 0)
                throw new ArgumentException("Capacity must be positive.");
            graphSeries = new List<Series>(initialSeriesCapacity);
        }

        int IndexOf(string name)
        {
            for (int i = graphSeries.Count - 1; i >= 0; --i)
            {
                if (graphSeries[i].Name == name)
                    return i;
            }
            return -1;
        }
        int IndexOf(IDataSeries data)
        {
            for (int i = graphSeries.Count - 1; i >= 0; --i)
            {
                if (graphSeries[i].Data == data)
                    return i;
            }
            return -1;
        }
        public Series GetSeries(string name)
        {
            var index = IndexOf(name);
            if (index >= 0)
                return graphSeries[index];
            throw new ArgumentException("No series with the given name exists within the graph.");
        }
        public Series GetSeries(IDataSeries data)
        {
            var index = IndexOf(data);
            if (index >= 0)
                return graphSeries[index];
            throw new ArgumentException("No series with the given data exists within the graph.");
        }

        public void AddSeries(string name, Vector3 lineColor, float lineRadius, IDataSeries series)
        {
            graphSeries.Add(new Series { Name = name, Data = series, LineRadius = lineRadius, LineColor = lineColor });
        }

        public void RemoveSeries(string name)
        {
            var index = IndexOf(name);
            if (index >= 0)
                graphSeries.RemoveAt(index);
            else
                throw new ArgumentException("No series with the given name exists within the graph.");
        }
        public void RemoveSeries(IDataSeries data)
        {
            var index = IndexOf(data);
            if (index >= 0)
                graphSeries.RemoveAt(index);
            else
                throw new ArgumentException("No series with the given data exists within the graph.");
        }

        public void ClearSeries()
        {
            graphSeries.Clear();
        }

        public void Draw(TextBuilder characters, UILineBatcher lines, TextBatcher text)
        {
            //Collect information to define data window ranges.
            int minX = int.MaxValue;
            int maxX = int.MinValue;
            var minY = double.MaxValue;
            var maxY = double.MinValue;
            for (int i = 0; i < graphSeries.Count; ++i)
            {
                var data = graphSeries[i].Data;
                if (minX > data.Start)
                {
                    minX = data.Start;
                }
                if (maxX < data.End)
                {
                    maxX = data.End;
                }
                for (int j = data.Start; j < data.End; ++j)
                {
                    var value = data[j];
                    if (minY > value)
                    {
                        minY = value;
                    }
                    if (maxY < value)
                    {
                        maxY = value;
                    }
                }
            }
            //If no data series contain values, then just use a default size.
            if (minY == float.MinValue)
            {
                minY = 0;
                maxY = 1;
            }
            //You could make use of this earlier to avoid comparisons but it doesn't really matter!
            if (description.ForceVerticalAxisMinimumToZero)
            {
                minY = 0;
                if (maxY < 0)
                    maxY = 1;
            }

            //Calculate the data span that takes into account rounding. We want intervals to be evenly spaced, but also to match nicely rounded numbers. 
            //That means the span must be equal to some rounded number multiplied by the number of intervals.
            var yDataSpan = maxY - minY;
            var yIntervalCount = description.TargetVerticalTickCount + 1;
            var rawIntervalLength = yDataSpan / yIntervalCount;
            var roundingOffset = 0.5 * Math.Pow(0.1, description.VerticalIntervalLabelRounding);
            yDataSpan =  Math.Round(rawIntervalLength * description.VerticalIntervalValueScale + roundingOffset, description.VerticalIntervalLabelRounding) * 
                (yIntervalCount / description.VerticalIntervalValueScale);

            //Draw the graph body axes.
            var lowerLeft = description.BodyMinimum + new Vector2(0, description.BodySpan.Y);
            var upperRight = description.BodyMinimum + new Vector2(description.BodySpan.X, 0);
            var lowerRight = description.BodyMinimum + description.BodySpan;
            lines.Draw(description.BodyMinimum, lowerLeft, description.AxisLineRadius, description.BodyLineColor);
            lines.Draw(lowerLeft, lowerRight, description.AxisLineRadius, description.BodyLineColor);

            //Draw axis labels.
            characters.Clear().Append(description.HorizontalAxisLabel);
            var baseAxisLabelDistance = description.IntervalTickLength + description.IntervalTextHeight * description.LineSpacingMultiplier;
            var verticalAxisLabelDistance = baseAxisLabelDistance + 2 * description.VerticalTickTextPadding;
            var horizontalAxisLabelDistance = baseAxisLabelDistance + 2 * description.HorizontalTickTextPadding + description.AxisLabelHeight * description.LineSpacingMultiplier;
            text.Write(characters,
                lowerLeft +
                new Vector2((description.BodySpan.X - GlyphBatch.MeasureLength(characters, description.Font, description.AxisLabelHeight)) * 0.5f, horizontalAxisLabelDistance),
                description.AxisLabelHeight, description.TextColor, description.Font);
            characters.Clear().Append(description.VerticalAxisLabel);
            text.Write(characters,
                description.BodyMinimum +
                new Vector2(-verticalAxisLabelDistance, (description.BodySpan.Y + GlyphBatch.MeasureLength(characters, description.Font, description.AxisLabelHeight)) * 0.5f),
                description.AxisLabelHeight, new Vector2(0, -1), description.TextColor, description.Font);

            //Position tickmarks, tick labels, and background lines along the axes.
            {
                var xDataIntervalSize = (maxX - minX) / (description.TargetHorizontalTickCount + 1f);
                var previousTickValue = int.MinValue;
                float valueToPixels = description.BodySpan.X / (maxX - minX);
                for (int i = 0; i < description.TargetHorizontalTickCount + 2; ++i)
                {
                    //Round pen offset such that the data tick lands on an integer.
                    var valueAtTick = i * xDataIntervalSize;
                    var tickValue = (int)Math.Round(valueAtTick);
                    if (tickValue == previousTickValue)
                    {
                        //Don't bother creating redundant ticks.
                        continue;
                    }
                    previousTickValue = tickValue;

                    var penPosition = lowerLeft + new Vector2(tickValue * valueToPixels, 0);
                    var tickEnd = penPosition + new Vector2(0, description.IntervalTickLength);
                    var backgroundEnd = penPosition - new Vector2(0, description.BodySpan.Y);
                    lines.Draw(penPosition, tickEnd, description.IntervalTickRadius, description.BodyLineColor);
                    lines.Draw(penPosition, backgroundEnd, description.BackgroundLineRadius, description.BodyLineColor);
                    characters.Clear().Append(tickValue);
                    text.Write(characters, tickEnd +
                        new Vector2(GlyphBatch.MeasureLength(characters, description.Font, description.IntervalTextHeight) * -0.5f,
                        description.HorizontalTickTextPadding + description.IntervalTextHeight * description.LineSpacingMultiplier),
                        description.IntervalTextHeight, description.TextColor, description.Font);
                }
            }
            {
                var yDataIntervalSize = yDataSpan / (description.TargetVerticalTickCount + 1f);
                var previousTickValue = double.MinValue;
                //Note the inclusion of the scale. Rounding occurs post-scale; moving back to pixels requires undoing the scale.
                var valueToPixels = description.BodySpan.Y / (yDataSpan * description.VerticalIntervalValueScale);
                for (int i = 0; i < description.TargetVerticalTickCount + 2; ++i)
                {
                    var tickValue = Math.Round((minY + yDataIntervalSize * i) * description.VerticalIntervalValueScale, description.VerticalIntervalLabelRounding);
                    if (tickValue == previousTickValue)
                    {
                        //Don't bother creating redundant ticks.
                        continue;
                    }
                    previousTickValue = tickValue;

                    var penPosition = lowerLeft - new Vector2(0, (float)(tickValue * valueToPixels));

                    var tickEnd = penPosition - new Vector2(description.IntervalTickLength, 0);
                    var backgroundEnd = penPosition + new Vector2(description.BodySpan.X, 0);
                    lines.Draw(penPosition, tickEnd, description.IntervalTickRadius, description.BodyLineColor);
                    lines.Draw(penPosition, backgroundEnd, description.BackgroundLineRadius, description.BodyLineColor);
                    characters.Clear().Append(tickValue, description.VerticalIntervalLabelRounding);
                    text.Write(characters,
                        tickEnd + new Vector2(-description.VerticalTickTextPadding, 0.5f * GlyphBatch.MeasureLength(characters, description.Font, description.IntervalTextHeight)),
                        description.IntervalTextHeight, new Vector2(0, -1), description.TextColor, description.Font);
                }
            }

            //Draw the line graphs on top of the body.
            {
                var dataToPixelsScale = new Vector2(description.BodySpan.X / (maxX - minX), (float)(description.BodySpan.Y / yDataSpan));
                Vector2 DataToScreenspace(int x, double y)
                {
                    var graphCoordinates = new Vector2(x - minX, (float)(y - minY)) * dataToPixelsScale;
                    var screenCoordinates = graphCoordinates;
                    screenCoordinates.Y = description.BodySpan.Y - screenCoordinates.Y;
                    screenCoordinates += description.BodyMinimum;
                    return screenCoordinates;
                }

                for (int i = 0; i < graphSeries.Count; ++i)
                {
                    var series = graphSeries[i];
                    var data = series.Data;
                    var count = data.End - data.Start;
                    if (count > 0)
                    {
                        var previousScreenPosition = DataToScreenspace(data.Start, data[data.Start]);
                        if (count > 1)
                        {
                            for (int j = data.Start + 1; j < data.End; ++j)
                            {
                                var currentScreenPosition = DataToScreenspace(j, data[j]);
                                lines.Draw(previousScreenPosition, currentScreenPosition, series.LineRadius, series.LineColor);
                                previousScreenPosition = currentScreenPosition;
                            }
                        }
                        else
                        {
                            //Only one datapoint. Draw a zero length line just to draw a dot. (The shader can handle it without producing nans.)
                            lines.Draw(previousScreenPosition, previousScreenPosition, series.LineRadius, series.LineColor);
                        }
                    }
                }
            }

            //Draw the legend entry last. Alpha blending will put it on top in case the legend is positioned on top of the body.
            {
                var penPosition = description.LegendMinimum;
                var legendLineSpacing = description.LegendNameHeight * 1.5f;
                penPosition.Y += legendLineSpacing;

                for (int i = 0; i < graphSeries.Count; ++i)
                {
                    var series = graphSeries[i];
                    var lineStart = new Vector2(penPosition.X, penPosition.Y);
                    var lineEnd = lineStart + new Vector2(description.LegendLineLength, -0.7f * description.LegendNameHeight);

                    lines.Draw(lineStart, lineEnd, series.LineRadius, series.LineColor);
                    var textStart = new Vector2(lineEnd.X + series.LineRadius + description.LegendNameHeight * 0.2f, penPosition.Y);
                    characters.Clear().Append(series.Name);
                    text.Write(characters, textStart, description.LegendNameHeight, description.TextColor, description.Font);
                    penPosition.Y += legendLineSpacing;
                }
            }
        }

    }
}
