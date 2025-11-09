#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QVector>
#include <QPair>
#include <QStringListModel>
#include <QStandardItemModel>
#include <QDir>
#include <QFile>
#include <QMap>
#include "qcustomplot.h"
#include "realtimewindow.h"
#include <limits>
#include <algorithm>
#include <cmath>

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

/**
 * @class MainWindow
 * @brief Main application window for power and energy monitoring system
 *
 * Provides comprehensive interface for monitoring CPU and GPU power consumption
 * and energy usage. Handles configuration management, real-time monitoring,
 * archive data visualization, and interactive plotting.
 */
class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    /**
     * @brief Constructor - creates main application window
     * @param parent Parent widget
     */
    MainWindow(QWidget *parent = nullptr);

    /**
     * @brief Destructor - cleans up UI resources
     */
    ~MainWindow();

private slots:
    // Configuration Management
    /**
     * @brief Applies current UI configuration and saves to file
     */
    void applyConfig();

    /**
     * @brief Opens file dialog to select an executable for monitoring
     */
    void loadExecutable();

    /**
     * @brief Saves current UI configuration to file
     */
    void on_saveButton_clicked();

    /**
     * @brief Loads configuration from selected file
     */
    void on_loadConfigButton_clicked();

    /**
     * @brief Clears all UI fields and resets configuration state
     */
    void on_clearButton_clicked();

    // Monitoring Control
    /**
     * @brief Starts the external monitoring process and real-time visualization
     */
    void on_startButton_clicked();

    /**
     * @brief Stops the running monitoring process and cleans up resources
     */
    void on_stopButton_clicked();

    // Data Loading and Visualization
    /**
     * @brief Loads and displays archive data from file
     */
    void on_loadArchiveButton_clicked();

    /**
     * @brief Updates the main plot based on current selection and display options
     */
    void updatePlotForSelection();

    /**
     * @brief Periodic UI update slot for real-time data visualization
     */
    void realtimeDataSlot();

    /**
     * @brief Rebuilds the subdomain list model based on current configuration
     */
    void updateListViewFromUseAndConfig();

    // UI Event Handlers
    /**
     * @brief Handles changes to the device type selection (USO combobox)
     * @param text The new selection text
     */
    void onUseBoxChanged(const QString &text);

    /**
     * @brief Displays application information dialog
     */
    void on_aboutButton_clicked();

    /**
     * @brief Placeholder for plot zoom/pan change events
     */
    void onPlotZoomOrPanChanged();

    /**
     * @brief Handles plot double-click to reset to original ranges
     */
    void onPlotDoubleClick();

    /**
     * @brief Handles plot double-click event to reset view
     */
    void onCustomPlotDoubleClick();

    // Axis Management
    /**
     * @brief Enforces positive Y-axis range and data boundaries
     * @param newRange The proposed new axis range
     */
    void enforcePositiveYAxis(const QCPRange &newRange);

    /**
     * @brief Enforces positive X-axis range and data boundaries
     * @param newRange The proposed new axis range
     */
    void enforcePositiveXAxis(const QCPRange &newRange);

private:
    // UI Components
    Ui::MainWindow *ui;                          ///< Pointer to UI elements
    QStandardItemModel *subdomainModel;          ///< Model for subdomain list view

    // Configuration Management
    QString currentConfigPath;                   ///< Current configuration file path
    QVector<QPair<QString, QString>> currentConfig;  ///< Current configuration data

    // Process Management
    QProcess *runningProcess;                    ///< Pointer to running monitoring process
    RealtimeWindow *realtimeWindow;              ///< Pointer to real-time monitoring window
    QTimer dataTimer;                            ///< Timer for periodic UI updates

    // Data Storage
    QMap<QString, QVector<double>> allTimestamps;    ///< Timestamp data per device
    QMap<QString, QVector<double>> allEnergyData;    ///< Energy data per device
    QMap<QString, QVector<double>> allPowerData;     ///< Power data per device
    QMap<QString, QVector<double>> accumulatedEnergy; ///< Accumulated energy per device

    // Plot Management
    QMap<QString, QCPGraph*> deviceGraphs;       ///< Graph objects per device
    QMap<QString, QColor> deviceColors;          ///< Color assignments per device

    // Data Range Tracking
    double m_dataXMin;                           ///< Minimum X value in data
    double m_dataXMax;                           ///< Maximum X value in data
    double m_dataYMin;                           ///< Minimum Y value in data
    double m_dataYMax;                           ///< Maximum Y value in data
    bool m_hasDataRange;                         ///< Flag indicating valid data range

    double originalXMin, originalXMax;           ///< Original X-axis range
    double originalYMin, originalYMax;           ///< Original Y-axis range

    // File Management
    QString lastLoadedArchivePath;               ///< Last loaded archive file path

    // Color Management
    QMap<QString, QColor> deviceColorMap;        ///< Unified color map for devices
    QVector<QColor> predefinedColors;            ///< Predefined color palette
    int nextColorIndex;                          ///< Index for next color assignment

    // Private Methods
    // Configuration File Operations
    /**
     * @brief Parses a configuration file into key-value pairs
     * @param filePath Path to the configuration file
     * @return QVector of key-value pairs representing the configuration
     */
    QVector<QPair<QString, QString>> parseConfig(const QString &filePath);

    /**
     * @brief Writes configuration key-value pairs to disk
     * @param filePath Destination file path
     * @param config Configuration data to write
     */
    void writeConfig(const QString &filePath, const QVector<QPair<QString, QString>> &config);

    /**
     * @brief Loads and applies configuration from specified file path
     * @param filePath Path to the configuration file
     */
    void loadConfigFromPath(const QString &filePath);

    /**
     * @brief Loads and parses archive data from specified file path
     * @param filePath Path to the archive data file
     */
    void loadArchiveFromPath(const QString &filePath);

    // Device and Color Management
    /**
     * @brief Retrieves device list from current configuration
     * @return QStringList of device identifiers
     */
    QStringList getDevicesFromConfig() const;

    /**
     * @brief Ensures consistent color assignment for device identifiers
     * @param id Device identifier to assign color to
     */
    void ensureColorForId(const QString &id);

    /**
     * @brief Retrieves assigned color for device identifier
     * @param deviceId Device identifier to get color for
     * @return QColor assigned to the device, or default gray if not found
     */
    QColor getDeviceColor(const QString &deviceId) const;

    // Validation and Debug
    /**
     * @brief Validates RAPL subdomains against system configuration
     * @param subdomainText Comma-separated list of subdomain names to validate
     * @param failedToken Output parameter for the first invalid token found
     * @param reason Output parameter for detailed failure reason
     * @return true if all subdomains are valid, false otherwise
     */
    bool validateSubdomains(const QString &subdomainText, QString &failedToken, QString &reason) const;

    /**
     * @brief Debug function to display available RAPL domains and naming formats
     */
    void debugRAPLDomains();
};

#endif // MAINWINDOW_H
