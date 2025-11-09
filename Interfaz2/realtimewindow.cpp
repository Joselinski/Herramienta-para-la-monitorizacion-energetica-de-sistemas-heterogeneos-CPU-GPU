#include "realtimewindow.h"
#include <QVBoxLayout>
#include <QPushButton>
#include <QBrush>
#include <QColor>
#include <QPen>

/**
 * @brief Constructs real-time monitoring window
 *
 * Initializes dual-plot interface with power and energy visualization,
 * sets up UI layout, and connects control signals.
 *
 * @param parent Parent widget
 */
RealtimeWindow::RealtimeWindow(QWidget *parent)
    : QWidget(parent)
{
    // Create main layout
    QVBoxLayout *layout = new QVBoxLayout(this);

    // Initialize plot widgets
    plotWWidget = new QCustomPlot(this);
    plotJWidget = new QCustomPlot(this);

    // Configure plots
    setupPlot(plotWWidget, "Power (W)");
    setupPlot(plotJWidget, "Energy (J)");

    // Add plots to layout
    layout->addWidget(plotWWidget);
    layout->addWidget(plotJWidget);

    // Create and configure stop button
    pauseButton = new QPushButton("Stop", this);
    layout->addWidget(pauseButton);

    // Connect stop button to signal
    connect(pauseButton, &QPushButton::clicked, this, [this]() {
        emit stopRequested();
    });

    // Finalize window setup
    setLayout(layout);
    setWindowTitle("Real-time Monitoring");
    setMinimumSize(800, 600);
    resize(800, 600);
}

/**
 * @brief Configures QCustomPlot widget for real-time display
 *
 * Sets up axis labels, legend, and initial ranges for consistent
 * real-time data visualization.
 *
 * @param plot Plot widget to configure
 * @param yLabel Y-axis label text
 */
void RealtimeWindow::setupPlot(QCustomPlot *plot, const QString &yLabel)
{
    plot->legend->setVisible(true);
    plot->legend->setBrush(QBrush(QColor(255, 255, 255, 150)));
    plot->xAxis->setLabel("Time (ms)");
    plot->yAxis->setLabel(yLabel);
    plot->xAxis->setRange(0, 10);
    plot->yAxis->setRange(0, 100);
}

/**
 * @brief Initializes plots and data structures for monitoring devices
 *
 * Clears previous data, creates graphs for each device with assigned colors,
 * and prepares the visualization system for new data streams.
 *
 * @param devices List of device identifiers to monitor
 * @param deviceColors Color mapping for device visualization
 */
void RealtimeWindow::setupDevices(const QStringList &devices, const QMap<QString, QColor> &deviceColors)
{
    // Store device list and color mapping
    deviceList = devices;
    this->deviceColors = deviceColors;

    // Clear previous data
    timeData.clear();
    powerData.clear();
    energyData.clear();
    accumulatedEnergy.clear();

    // Clear existing graphs
    plotWWidget->clearGraphs();
    plotJWidget->clearGraphs();

    // Initialize data structures and create graphs for each device
    for (const QString &dev : deviceList)
    {
        // Initialize data containers
        timeData[dev].clear();
        powerData[dev].clear();
        energyData[dev].clear();
        accumulatedEnergy[dev] = 0.0;

        // Get assigned color (default to gray if not specified)
        QColor color = deviceColors.value(dev, QColor(128, 128, 128));

        // Create power graph
        QCPGraph *g1 = plotWWidget->addGraph();
        g1->setName(dev);
        g1->setPen(QPen(color, 2));

        // Create energy graph
        QCPGraph *g2 = plotJWidget->addGraph();
        g2->setName(dev);
        g2->setPen(QPen(color, 2));
    }

    // Refresh plots
    plotWWidget->replot();
    plotJWidget->replot();
}

/**
 * @brief Adds new data point to real-time visualization
 *
 * Updates internal data structures, recalculates accumulated energy,
 * and refreshes both power and energy plots with new data.
 *
 * @param timestamp Time value in milliseconds
 * @param powerValues Power measurements keyed by device identifier
 */
void RealtimeWindow::addDataPoint(double timestamp, const QMap<QString, double> &powerValues)
{
    // Update data structures for each device
    for (auto it = powerValues.constBegin(); it != powerValues.constEnd(); ++it)
    {
        QString dev = it.key();
        double p = it.value();

        if (!timeData.contains(dev)) continue;

        // Add new data point
        timeData[dev].append(timestamp);
        powerData[dev].append(p);

        // Calculate accumulated energy (simplified integration)
        accumulatedEnergy[dev] += p;
        energyData[dev].append(accumulatedEnergy[dev]);
    }

    // Update all graphs with new data
    for (int i = 0; i < deviceList.size(); ++i)
    {
        const QString &dev = deviceList[i];
        if (!timeData.contains(dev)) continue;

        plotWWidget->graph(i)->setData(timeData[dev], powerData[dev]);
        plotJWidget->graph(i)->setData(timeData[dev], energyData[dev]);
    }

    // Auto-scale axes to fit new data
    plotWWidget->rescaleAxes();
    plotJWidget->rescaleAxes();

    // Refresh visualization
    plotWWidget->replot();
    plotJWidget->replot();
}

/**
 * @brief Stops real-time updates
 *
 * Placeholder function for stopping background timers or other
 * update mechanisms that may be added in the future.
 */
void RealtimeWindow::stopUpdates()
{
    // If using timers, they could be stopped here
}
