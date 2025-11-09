#ifndef REALTIMEWINDOW_H
#define REALTIMEWINDOW_H

#include <QWidget>
#include "qcustomplot.h"
#include <QVBoxLayout>
#include <QMap>
#include <QPushButton>

/**
 * @class RealtimeWindow
 * @brief Window for real-time monitoring of power and energy data
 *
 * Displays two synchronized plots:
 * - Power consumption in Watts (instantaneous)
 * - Energy consumption in Joules (accumulated)
 *
 * Provides real-time visualization of monitoring data with
 * consistent color coding across devices.
 */
class RealtimeWindow : public QWidget
{
    Q_OBJECT

public:
    /**
     * @brief Constructor - creates real-time monitoring window
     * @param parent Parent widget
     */
    explicit RealtimeWindow(QWidget *parent = nullptr);

    /**
     * @brief Initializes plots for specified monitoring devices
     * @param devices List of device identifiers to monitor
     * @param deviceColors Color mapping for each device
     */
    void setupDevices(const QStringList &devices, const QMap<QString, QColor> &deviceColors);

    /**
     * @brief Adds new data point to real-time plots
     * @param timestamp Time value in milliseconds
     * @param powerValues Power measurements for each device
     */
    void addDataPoint(double timestamp, const QMap<QString, double> &powerValues);

    /**
     * @brief Stops real-time updates (placeholder for future enhancements)
     */
    void stopUpdates();

signals:
    /**
     * @brief Signal emitted when stop button is pressed
     */
    void stopRequested();

private:
    // Plot widgets
    QCustomPlot *plotWWidget;    ///< Power plot (Watts)
    QCustomPlot *plotJWidget;    ///< Energy plot (Joules)

    // UI controls
    QPushButton *pauseButton;    ///< Stop monitoring button

    // Data storage
    QStringList deviceList;      ///< List of monitored devices

    // Time series data per device
    QMap<QString, QVector<double>> timeData;      ///< Timestamp values
    QMap<QString, QVector<double>> powerData;     ///< Power values (W)
    QMap<QString, QVector<double>> energyData;    ///< Energy values (J)
    QMap<QString, double> accumulatedEnergy;      ///< Running energy total

    // Visualization settings
    QMap<QString, QColor> deviceColors;  ///< Color assignment per device

    /**
     * @brief Configures a QCustomPlot widget for real-time display
     * @param plot Plot widget to configure
     * @param yLabel Y-axis label text
     */
    void setupPlot(QCustomPlot *plot, const QString &yLabel);
};

#endif // REALTIMEWINDOW_H
