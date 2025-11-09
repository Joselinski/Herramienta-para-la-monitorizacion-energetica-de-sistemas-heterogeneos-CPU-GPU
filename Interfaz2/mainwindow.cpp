/******************************************************************************
 * mainwindow.cpp
 *
 * Main window implementation for the monitoring application.
 *
 * This file implements:
 *  - UI wiring and initialization (constructor / destructor)
 *  - Loading / saving of a simple key=value configuration file
 *  - Starting an external monitoring process (QProcess) and parsing its
 *    standard output for real-time plotting in a separate RealtimeWindow.
 *  - Loading offline output files into the main plot (archive mode)
 *  - Helpers to build the UI lists and manage QCustomPlot graphs
 *
 * Bachelor's Thesis Implementation
 * Author: José Javier Pérez Sánchez
 * Supervised by: Sergio Santander Jiménez and José María Granado Criado
 ******************************************************************************/

#include "mainwindow.h"
#include "ui_mainwindow.h"

#include <QFileDialog>
#include <QFile>
#include <QTextStream>
#include <QMessageBox>
#include <QStandardItemModel>
#include <QStandardItem>
#include <QProcess>
#include <QDebug>
#include <QDir>
#include <QFile>
#include <QMap>
#include <algorithm>
#include <limits>
#include <cmath>

//=============================================================================
// CONSTRUCTION / DESTRUCTION
//=============================================================================

/**
 * @brief Constructs the main application window
 *
 * Initializes all UI components, establishes signal-slot connections,
 * configures the plotting system, and sets up default application state.
 *
 * @param parent Parent widget (typically nullptr for main window)
 */
MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    setWindowTitle("Monitorizing Application");

    // ==================== UI COMPONENT INITIALIZATION ====================

    // Initialize unified color management system
    deviceColorMap = QMap<QString, QColor>();

    // Initialize data range tracking variables
    m_hasDataRange = false;
    m_dataXMin = 0; m_dataXMax = 0;
    m_dataYMin = 0; m_dataYMax = 0;

    // Configure subdomain list model with checkable items
    subdomainModel = new QStandardItemModel(this);
    ui->listView->setModel(subdomainModel);
    ui->listView->setSelectionMode(QAbstractItemView::NoSelection);

    // ==================== SIGNAL-SLOT CONNECTIONS ====================

    // Configuration buttons
    connect(ui->applyButton, &QPushButton::clicked, this, &MainWindow::applyConfig);
    connect(ui->executableButton, &QPushButton::clicked, this, &MainWindow::loadExecutable);
    connect(ui->useBox, &QComboBox::currentTextChanged, this, &MainWindow::onUseBoxChanged);

    // Plot interactions
    connect(ui->customPlot->xAxis, SIGNAL(rangeChanged(QCPRange)),
            this, SLOT(onPlotZoomOrPanChanged()));
    connect(ui->customPlot, &QCustomPlot::mouseDoubleClick,
            this, &MainWindow::onCustomPlotDoubleClick);

    // Enable drag and zoom for plot
    ui->customPlot->axisRect()->setRangeDrag(Qt::Horizontal | Qt::Vertical);
    ui->customPlot->axisRect()->setRangeZoom(Qt::Horizontal | Qt::Vertical);

    // ==================== PLOTTING SYSTEM CONFIGURATION ====================

    ui->customPlot->clearGraphs();
    ui->customPlot->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom);

    // Axis range enforcement to prevent negative values
    connect(ui->customPlot->xAxis, SIGNAL(rangeChanged(QCPRange)),
            this, SLOT(enforcePositiveXAxis(QCPRange)));
    connect(ui->customPlot->yAxis, SIGNAL(rangeChanged(QCPRange)),
            this, SLOT(enforcePositiveYAxis(QCPRange)));

    // Configure plot appearance
    ui->customPlot->xAxis->setLabel("Time (ms)");
    ui->customPlot->yAxis->setLabel("Power (W)");
    ui->customPlot->legend->setVisible(true);
    ui->customPlot->legend->setBrush(QBrush(QColor(255, 255, 255, 150)));

    // Set initial axis ranges
    ui->customPlot->xAxis->setRange(0, 2000);
    ui->customPlot->yAxis->setRange(0, 100);
    ui->customPlot->replot();

    // ==================== UI STATE INITIALIZATION ====================

    // Set default checkbox states
    ui->powerCheck->setChecked(true);
    ui->energyCheck->setChecked(false);

    // Connect energy/power checkboxes to plot updates
    connect(ui->energyCheck, &QCheckBox::toggled, this, [this](bool){
        updatePlotForSelection();
    });
    connect(ui->powerCheck, &QCheckBox::toggled, this, [this](bool){
        updatePlotForSelection();
    });

    // Populate useBox with default options if empty
    if (ui->useBox->count() == 0) {
        ui->useBox->addItems({"CPU", "GPU", "CPU,GPU"});
    }

    // ==================== TIMER CONFIGURATION ====================

    // Set up timer for periodic UI updates (non-blocking)
    connect(&dataTimer, &QTimer::timeout, this, &MainWindow::realtimeDataSlot);
    dataTimer.start(500); // Update every 500ms

    // ==================== DEBUG AND VALIDATION ====================

    // Perform initial RAPL domain discovery and validation
    //debugRAPLDomains();
}

/**
 * @brief Destructor - cleans up UI resources
 */
MainWindow::~MainWindow()
{
    delete ui;
}

//=============================================================================
// CONFIGURATION MANAGEMENT
//=============================================================================

/**
 * @brief Applies current UI configuration and saves to file
 *
 * Validates all input fields, saves configuration to disk, updates the
 * subdomain list model, and refreshes the plot. Shows appropriate error
 * messages for invalid inputs.
 */
void MainWindow::applyConfig()
{
    // Request save location if no current config path exists
    if (currentConfigPath.isEmpty()) {
        QString filePath = QFileDialog::getSaveFileName(
            this, tr("Save Configuration File"), "",
            tr("Config Files (*.conf)"));
        if (filePath.isEmpty()) return;
        if (!filePath.endsWith(".conf", Qt::CaseInsensitive))
            filePath += ".conf";
        currentConfigPath = filePath;
        ui->lineEdit->setText(filePath);
    }

    // --- Validate numeric fields ---
    bool okSample, okPost, okMin;
    double sampleTime = ui->sampleTimeEdit->text().toDouble(&okSample);
    double postExec   = ui->postExecEdit->text().toDouble(&okPost);
    double minPower   = ui->minPowerEdit->text().toDouble(&okMin);

    if (!okSample || sampleTime < 1) {
        QMessageBox::warning(this, "Invalid Value",
                             "Sample time must be a number greater than or equal to 1.");
        return;
    }
    if (!okPost || postExec < 0) {
        QMessageBox::warning(this, "Invalid Value",
                             "Post-execution time must be a number greater than or equal to 0.");
        return;
    }
    if (!okMin || minPower < 0) {
        QMessageBox::warning(this, "Invalid Value",
                             "Minimum power must be a number greater than or equal to 0.");
        return;
    }

    // --- Validate executable ---
    QFileInfo execFile(ui->executableEdit->text());
    if (!execFile.exists() || !execFile.isExecutable()) {
        QMessageBox::warning(this, "Invalid Executable",
                             "The specified executable file does not exist or is not executable.");
        return;
    }

    // --- Validate RAPL subdomains ---
    QStringList invalidSubdomains;
    QString subdomainText = ui->subdomainEdit->text();

    if (!subdomainText.isEmpty()) {
        QString failedToken, reason;
        if (!validateSubdomains(subdomainText, failedToken, reason)) {
            invalidSubdomains << failedToken;
        }
    }

    // --- Validate GPUs --- (Commented out temporarily)
    QStringList invalidGpus;
    /* QString gpuText = ui->gpuEdit->text();
    if (!gpuText.isEmpty()) {
        QString failedToken, reason;
        if (!validateGpus(gpuText, failedToken, reason)) {
            invalidGpus << failedToken;
        }
    } */

    // --- Warn if invalid devices found ---
    if (!invalidSubdomains.isEmpty() || !invalidGpus.isEmpty()) {
        QString msg = "The following configuration elements are invalid:\n\n";
        if (!invalidSubdomains.isEmpty()) {
            QString failedToken, reason;
            validateSubdomains(subdomainText, failedToken, reason);
            msg += "Invalid RAPL subdomains: " + invalidSubdomains.join(", ") + "\n";
            msg += "Reason: " + reason + "\n\n";
        }
        if (!invalidGpus.isEmpty()) {
            // GPU validation details would go here
        }
        msg += "\nDo you want to continue applying the configuration anyway?";

        QMessageBox::StandardButton reply = QMessageBox::warning(
            this, "Invalid Configuration", msg,
            QMessageBox::Yes | QMessageBox::Cancel, QMessageBox::Cancel
        );

        if (reply == QMessageBox::Cancel)
            return;
    }

    // --- Save configuration ---
    QVector<QPair<QString, QString>> newConfig;
    newConfig.append(qMakePair(QString("SAMPLE_TIME"), ui->sampleTimeEdit->text()));
    newConfig.append(qMakePair(QString("POST_EXEC_TIME"), ui->postExecEdit->text()));
    newConfig.append(qMakePair(QString("MIN_POWER"), ui->minPowerEdit->text()));
    newConfig.append(qMakePair(QString("OUTPUT_FILE"), ui->outputFIleEdit->text()));
    newConfig.append(qMakePair(QString("SUBDOMAIN"), ui->subdomainEdit->text()));
    newConfig.append(qMakePair(QString("USO"), ui->useBox->currentText()));
    newConfig.append(qMakePair(QString("EXECUTABLE"), ui->executableEdit->text()));
    newConfig.append(qMakePair(QString("ARGUMENTS"), ui->argumentsEdit->text()));
    newConfig.append(qMakePair(QString("GPU"), ui->gpuEdit->text()));

    writeConfig(currentConfigPath, newConfig);
    currentConfig = newConfig;

    // --- Rebuild subdomain list ---
    subdomainModel->clear();

    QString uso = ui->useBox->currentText().trimmed().toUpper();
    QStringList cpuSubdomains = ui->subdomainEdit->text().split(",", Qt::SkipEmptyParts);
    QStringList gpuIds = ui->gpuEdit->text().split(",", Qt::SkipEmptyParts);

    // Add CPU subdomains to list
    if (uso == "CPU" || uso == "CPU,GPU") {
        for (const QString &sd : cpuSubdomains) {
            QString name = sd.trimmed();
            if (!name.isEmpty()) {
                QStandardItem *item = new QStandardItem(name);
                item->setEditable(false);
                item->setCheckable(true);
                item->setCheckState(Qt::Checked);
                subdomainModel->appendRow(item);
            }
        }
    }

    // Add GPU devices to list
    if (uso == "GPU" || uso == "CPU,GPU") {
        for (const QString &id : gpuIds) {
            QString trimmed = id.trimmed();
            if (!trimmed.isEmpty()) {
                QStandardItem *item = new QStandardItem("GPU" + trimmed);
                item->setEditable(false);
                item->setCheckable(true);
                item->setCheckState(Qt::Checked);
                subdomainModel->appendRow(item);
            }
        }
    }

    // Update UI components
    updateListViewFromUseAndConfig();
    updatePlotForSelection();

    QMessageBox::information(this, "Configuration Applied",
                             "The configuration file was successfully saved and applied.");
}

/**
 * @brief Parses a configuration file into key-value pairs
 *
 * Supports simple key=value format with # for comments and blank line skipping.
 * Preserves order of configuration entries.
 *
 * @param filePath Path to the configuration file
 * @return QVector of key-value pairs representing the configuration
 */
QVector<QPair<QString, QString>> MainWindow::parseConfig(const QString &filePath)
{
    QVector<QPair<QString, QString>> config;
    QFile file(filePath);

    if (file.open(QIODevice::ReadOnly | QIODevice::Text)) {
        QTextStream in(&file);
        while (!in.atEnd()) {
            QString line = in.readLine().trimmed();
            // Skip empty lines and comments
            if (line.isEmpty() || line.startsWith("#")) continue;

            QStringList parts = line.split("=");
            if (parts.size() == 2) {
                config.append(qMakePair(parts[0].trimmed(), parts[1].trimmed()));
            }
        }
        file.close();
    }
    return config;
}

/**
 * @brief Writes configuration key-value pairs to disk
 *
 * @param filePath Destination file path
 * @param config Configuration data to write
 */
void MainWindow::writeConfig(const QString &filePath, const QVector<QPair<QString, QString>> &config)
{
    QFile file(filePath);
    if (file.open(QIODevice::WriteOnly | QIODevice::Text)) {
        QTextStream out(&file);
        for (const auto &p : config) {
            out << p.first << "=" << p.second << "\n";
        }
        file.close();
    }
}

//=============================================================================
// MONITORING PROCESS MANAGEMENT
//=============================================================================

/**
 * @brief Starts the external monitoring process and real-time visualization
 *
 * Validates configuration, launches the monitoring binary via QProcess,
 * opens a real-time visualization window, and connects signal handlers
 * for process output and completion.
 */
void MainWindow::on_startButton_clicked()
{
    // Validate preconditions
    if (currentConfigPath.isEmpty()) {
        QMessageBox::warning(this, "Error", "Please load a configuration file first.");
        return;
    }

    if (runningProcess && runningProcess->state() == QProcess::Running) {
        QMessageBox::information(this, "Running", "A process is already running.");
        return;
    }

    // Disable main window during monitoring to prevent accidental interactions
    this->setEnabled(false);

    // Create and configure real-time window
    realtimeWindow = new RealtimeWindow();

    // Get device list and assign consistent colors
    QStringList devices = getDevicesFromConfig();
    QMap<QString, QColor> deviceColors;
    for (const QString &device : devices) {
        ensureColorForId(device);
        deviceColors[device] = getDeviceColor(device);
    }

    realtimeWindow->setupDevices(devices, deviceColors);
    realtimeWindow->show();

    // Connect stop signal from realtime window
    connect(realtimeWindow, &RealtimeWindow::stopRequested, this, [this]() {
        if (runningProcess) {
            runningProcess->kill();
            runningProcess->waitForFinished();
        }
    });

    // Extract command components from configuration
    QString executable;
    QString arguments;
    QString outputFile;

    for (auto &p : currentConfig) {
        if (p.first == "EXECUTABLE") executable = p.second;
        else if (p.first == "ARGUMENTS") arguments = p.second;
        else if (p.first == "OUTPUT_FILE") outputFile = p.second;
    }

    // Build command: sudo <appdir>/final <config> <executable> [args]
    QString finalPath = QCoreApplication::applicationDirPath() + "/final";
    QString program = "sudo";
    QStringList params;
    params << finalPath << currentConfigPath << executable;
    if (!arguments.isEmpty()) params << arguments;

    // Create and configure monitoring process
    runningProcess = new QProcess(this);
    runningProcess->setProgram(program);
    runningProcess->setArguments(params);

    // Real-time output parsing for process stdout
    connect(runningProcess, &QProcess::readyReadStandardOutput, this, [this]() {
        QString output = runningProcess->readAllStandardOutput();
        QStringList lines = output.split("\n", Qt::SkipEmptyParts);

        for (const QString &line : lines) {
            QStringList parts = line.split(";", Qt::KeepEmptyParts);
            if (parts.size() < 5) continue;

            bool ok;
            double t = parts[0].toDouble(&ok); if (!ok) continue;
            QString dev = parts[1].trimmed();
            QString id = parts[2].trimmed();
            double power  = parts[4].toDouble(&ok); if (!ok) power = 0.0;

            // Map process identifiers to UI keys
            QString devKey = (dev == "CPU") ? id : "GPU" + id;

            QMap<QString, double> powerValues;
            powerValues[devKey] = power;

            // Forward to real-time window for visualization
            if (realtimeWindow)
                realtimeWindow->addDataPoint(t, powerValues);
        }
    });

    // Process completion handler
    connect(runningProcess, QOverload<int, QProcess::ExitStatus>::of(&QProcess::finished),
            this, [this, outputFile](int exitCode, QProcess::ExitStatus) {
        QMessageBox::information(this, "Execution finished",
                                 QString("Process exited with code %1").arg(exitCode));

        // Clean up real-time window
        if (realtimeWindow) {
            realtimeWindow->close();
            realtimeWindow->deleteLater();
            realtimeWindow = nullptr;
        }

        // Load output file if specified
        if (!outputFile.isEmpty())
            loadArchiveFromPath(outputFile);

        // Clean up process
        runningProcess->deleteLater();
        runningProcess = nullptr;

        // Re-enable main window
        this->setEnabled(true);
    });

    // Start the monitoring process
    runningProcess->start();
}

/**
 * @brief Opens file dialog to select an executable for monitoring
 *
 * Updates both the UI field and in-memory configuration with the selected
 * executable path.
 */
void MainWindow::loadExecutable()
{
    QString filePath = QFileDialog::getOpenFileName(
        this, "Select executable file", "", "Executable Files (*)");

    if (filePath.isEmpty()) return;

    ui->executableEdit->setText(filePath);

    // Update in-memory configuration
    bool found = false;
    for (auto &p : currentConfig) {
        if (p.first == "EXECUTABLE") {
            p.second = filePath;
            found = true;
            break;
        }
    }
    if (!found) {
        currentConfig.append(qMakePair(QString("EXECUTABLE"), filePath));
    }
}

//=============================================================================
// ARCHIVE DATA MANAGEMENT
//=============================================================================

/**
 * @brief Loads and displays archive data from file
 *
 * Clears current state, opens file dialog for output file selection,
 * parses the selected file, and updates the visualization.
 */
void MainWindow::on_loadArchiveButton_clicked()
{
    // Clear previous state
    ui->customPlot->clearGraphs();
    ui->customPlot->replot();

    ui->lineEdit->clear();
    ui->sampleTimeEdit->clear();
    ui->postExecEdit->clear();
    ui->minPowerEdit->clear();
    ui->outputFIleEdit->clear();
    ui->subdomainEdit->clear();
    ui->gpuEdit->clear();
    ui->executableEdit->clear();
    ui->argumentsEdit->clear();
    subdomainModel->clear();

    allTimestamps.clear();
    allEnergyData.clear();
    allPowerData.clear();
    accumulatedEnergy.clear();

    // Select output file (.out, .txt, or .csv)
    QString filePath = QFileDialog::getOpenFileName(
        this, tr("Select output file"), QString(),
        tr("Output Files (*.out *.txt *.csv);;All Files (*)"));

    if (filePath.isEmpty()) return;

    // Load and parse the selected file
    loadArchiveFromPath(filePath);
    lastLoadedArchivePath = filePath;

    // Check all items by default for convenience
    for (int row = 0; row < subdomainModel->rowCount(); ++row) {
        QStandardItem *item = subdomainModel->item(row);
        if (item)
            item->setCheckState(Qt::Checked);
    }
}

/**
 * @brief Loads configuration from selected file
 *
 * Opens file dialog and delegates to loadConfigFromPath for actual loading.
 */
void MainWindow::on_loadConfigButton_clicked()
{
    QString filePath = QFileDialog::getOpenFileName(
        this, tr("Select configuration file"), QString(),
        tr("Config Files (*.conf)"));

    if (!filePath.isEmpty()) {
        loadConfigFromPath(filePath);
    }
}

/**
 * @brief Loads and applies configuration from specified file path
 *
 * Clears current state, parses configuration file, populates UI fields,
 * and rebuilds the subdomain list model.
 *
 * @param filePath Path to the configuration file
 */
void MainWindow::loadConfigFromPath(const QString &filePath)
{
    if (filePath.isEmpty()) return;

    // Clear current state
    ui->customPlot->clearGraphs();
    ui->customPlot->replot();
    ui->archiveEdit->clear();

    ui->lineEdit->clear();
    ui->sampleTimeEdit->clear();
    ui->postExecEdit->clear();
    ui->minPowerEdit->clear();
    ui->outputFIleEdit->clear();
    ui->subdomainEdit->clear();
    ui->gpuEdit->clear();
    ui->executableEdit->clear();
    ui->argumentsEdit->clear();

    subdomainModel->clear();
    allTimestamps.clear();
    allEnergyData.clear();
    allPowerData.clear();
    accumulatedEnergy.clear();

    // Set current configuration path
    currentConfigPath = filePath;
    ui->lineEdit->setText(filePath);

    // Parse configuration file
    currentConfig = parseConfig(filePath);

    // Lambda helper for configuration value lookup
    auto getValue = [&](const QString &key) -> QString {
        for (auto &p : currentConfig) {
            if (p.first == key) return p.second;
        }
        return "";
    };

    // Populate UI fields with configuration values
    ui->sampleTimeEdit->setText(getValue("SAMPLE_TIME"));
    ui->postExecEdit->setText(getValue("POST_EXEC_TIME"));
    ui->minPowerEdit->setText(getValue("MIN_POWER"));
    ui->outputFIleEdit->setText(getValue("OUTPUT_FILE"));
    ui->subdomainEdit->setText(getValue("SUBDOMAIN"));
    ui->executableEdit->setText(getValue("EXECUTABLE"));
    ui->argumentsEdit->setText(getValue("ARGUMENTS"));

    // Set USO combobox
    QString usoValue = getValue("USO").trimmed().toUpper();
    int usoIndex = ui->useBox->findText(usoValue, Qt::MatchFixedString);
    ui->useBox->setCurrentIndex((usoIndex != -1) ? usoIndex : 0);

    // Set GPU IDs
    ui->gpuEdit->setText(getValue("GPU").trimmed());

    // Rebuild subdomain model
    subdomainModel->clear();
    QStringList cpuSubdomains = getValue("SUBDOMAIN").split(",", Qt::SkipEmptyParts);
    QStringList gpuIds = getValue("GPU").split(",", Qt::SkipEmptyParts);
    QSet<QString> addedItems;

    // Add CPU subdomains to model
    for (const QString &sd : cpuSubdomains) {
        QString name = sd.trimmed();
        if (!name.isEmpty() && !addedItems.contains(name)) {
            QStandardItem *item = new QStandardItem(name);
            item->setEditable(false);
            item->setCheckable(true);
            item->setCheckState(Qt::Checked);
            subdomainModel->appendRow(item);
            addedItems.insert(name);
        }
    }

    // Add GPU entries to model
    for (const QString &id : gpuIds) {
        QString trimmed = id.trimmed();
        if (!trimmed.isEmpty()) {
            QString gpuName = "GPU" + trimmed;
            if (!addedItems.contains(gpuName)) {
                QStandardItem *item = new QStandardItem(gpuName);
                item->setEditable(false);
                item->setCheckable(true);
                item->setCheckState(Qt::Checked);
                subdomainModel->appendRow(item);
                addedItems.insert(gpuName);
            }
        }
    }

    // Update visualization
    updateListViewFromUseAndConfig();
    updatePlotForSelection();
}

/**
 * @brief Saves current UI configuration to file
 *
 * Opens save dialog, writes all configuration parameters to file,
 * and immediately loads the saved configuration back.
 */
void MainWindow::on_saveButton_clicked()
{
    QString fileName = QFileDialog::getSaveFileName(
        this, tr("Save Configuration"), QDir::homePath() + "/new_config.conf",
        tr("Config Files (*.conf)"));

    if (fileName.isEmpty()) return;

    // Ensure .conf extension
    if (!fileName.endsWith(".conf", Qt::CaseInsensitive))
        fileName += ".conf";

    QFile file(fileName);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) {
        QMessageBox::warning(this, tr("Error"), tr("Cannot write configuration file."));
        return;
    }

    // Write configuration parameters from UI to file
    QTextStream out(&file);
    out << "SAMPLE_TIME="   << ui->sampleTimeEdit->text()           << "\n";
    out << "POST_EXEC_TIME="<< ui->postExecEdit->text()             << "\n";
    out << "MIN_POWER="     << ui->minPowerEdit->text()             << "\n";
    out << "OUTPUT_FILE="   << ui->outputFIleEdit->text()           << "\n";
    out << "SUBDOMAIN="     << ui->subdomainEdit->text()            << "\n";
    out << "USO="           << ui->useBox->currentText().trimmed()  << "\n";
    out << "GPU="           << ui->gpuEdit->text()                  << "\n";
    out << "EXECUTABLE="    << ui->executableEdit->text()           << "\n";
    out << "ARGUMENTS="     << ui->argumentsEdit->text()            << "\n";
    file.close();

    // Load the newly saved configuration into the UI
    loadConfigFromPath(fileName);

    QMessageBox::information(this, tr("Saved"),
                             tr("Configuration saved to %1 and loaded").arg(fileName));
}

//=============================================================================
// PLOTTING AND VISUALIZATION
//=============================================================================

/**
 * @brief Updates the main plot based on current selection and display options
 *
 * Rebuilds QCustomPlot graphs according to checked items in the subdomain list.
 * Supports switching between power and energy visualization modes.
 * Automatically adjusts axis ranges with appropriate padding.
 */
void MainWindow::updatePlotForSelection()
{
    ui->customPlot->clearGraphs();

    bool showEnergy = ui->energyCheck->isChecked();
    bool anyPlotted = false;

    // Initialize range tracking variables
    double xMin = std::numeric_limits<double>::infinity();
    double xMax = -std::numeric_limits<double>::infinity();
    double yMin = std::numeric_limits<double>::infinity();
    double yMax = -std::numeric_limits<double>::infinity();

    // Process each item in the subdomain model
    for (int row = 0; row < subdomainModel->rowCount(); ++row) {
        QStandardItem *item = subdomainModel->item(row);
        if (!item) continue;

        // Get device ID from UserRole or fallback to display text
        QString id = item->data(Qt::UserRole).toString();
        if (id.isEmpty()) {
            id = item->text();
        }

        // Ensure consistent color assignment
        ensureColorForId(id);

        // Get data vectors for this device
        QVector<double> x = allTimestamps.value(id);
        QVector<double> y = showEnergy ? accumulatedEnergy.value(id) : allPowerData.value(id);

        if (x.isEmpty() || y.isEmpty()) continue;

        // Create graph for checked items only
        if (item->checkState() == Qt::Checked) {
            QCPGraph *graph = ui->customPlot->addGraph();
            graph->setName(item->text());
            graph->setData(x, y);

            // Apply consistent color from device color map
            QColor color = deviceColorMap.value(id);
            graph->setPen(QPen(color, 2));

            anyPlotted = true;

            // Update data range boundaries
            auto localXMinIt = std::min_element(x.constBegin(), x.constEnd());
            auto localXMaxIt = std::max_element(x.constBegin(), x.constEnd());
            auto localYMinIt = std::min_element(y.constBegin(), y.constEnd());
            auto localYMaxIt = std::max_element(y.constBegin(), y.constEnd());

            if (localXMinIt != x.constEnd()) {
                xMin = std::min(xMin, *localXMinIt);
                xMax = std::max(xMax, *localXMaxIt);
            }
            if (localYMinIt != y.constEnd()) {
                yMin = std::min(yMin, *localYMinIt);
                yMax = std::max(yMax, *localYMaxIt);
            }

            qDebug() << "Gráfico dibujado - ID:" << id << "Color:" << color.name() << "Filas:" << x.size();
        }
    }

    // Handle empty plot case
    if (!anyPlotted) {
        m_hasDataRange = false;
        ui->customPlot->rescaleAxes();
        ui->customPlot->replot();
        ui->customPlot->yAxis->setLabel(showEnergy ? "Energy (J)" : "Power (W)");
        return;
    }

    // Calculate axis ranges with padding for better visualization
    double xSpan = xMax - xMin;
    if (xSpan <= 0.0) xSpan = 1.0;
    double xPad = std::max(xSpan * 0.02, 0.0);

    double ySpan = yMax - yMin;
    if (ySpan <= 0.0) ySpan = std::max(1.0, std::abs(yMax) * 0.1);
    double yPad = std::max(ySpan * 0.05, 1e-6);

    double newXLower = std::max(0.0, xMin - xPad);
    double newXUpper = xMax + xPad;
    double newYLower = std::max(0.0, yMin - yPad);
    double newYUpper = yMax + yPad;

    // Ensure minimum Y span for visibility
    double minYSpan = std::max(1e-3, std::abs(yMax) * 0.01);
    if (newYUpper - newYLower < minYSpan) {
        newYLower = std::max(0.0, yMax - minYSpan * 0.5);
        newYUpper = newYLower + minYSpan;
    }

    // Store calculated ranges for later use
    m_dataXMin = newXLower;
    m_dataXMax = newXUpper;
    m_dataYMin = newYLower;
    m_dataYMax = newYUpper;
    m_hasDataRange = true;

    // Apply calculated ranges and update plot
    ui->customPlot->xAxis->setRange(m_dataXMin, m_dataXMax);
    ui->customPlot->yAxis->setRange(m_dataYMin, m_dataYMax);
    ui->customPlot->yAxis->setLabel(showEnergy ? "Energy (J)" : "Power (W)");
    ui->customPlot->replot();
}

//=============================================================================
// AXIS RANGE MANAGEMENT
//=============================================================================

/**
 * @brief Enforces positive Y-axis range and data boundaries
 *
 * Prevents negative Y-values and maintains minimum span for visibility.
 * Connected to Y-axis rangeChanged signal.
 *
 * @param newRange The proposed new axis range
 */
void MainWindow::enforcePositiveYAxis(const QCPRange &newRange)
{
    QCPRange fixedRange = newRange;

    // Apply data boundaries if available
    if (m_hasDataRange) {
        // Clamp to data boundaries
        if (fixedRange.lower < m_dataYMin) fixedRange.lower = m_dataYMin;
        if (fixedRange.upper > m_dataYMax) fixedRange.upper = m_dataYMax;

        // Ensure minimum span for visibility
        double span = fixedRange.upper - fixedRange.lower;
        double minSpan = std::max(1e-6, m_dataYMax * 0.001);
        if (span < minSpan) {
            fixedRange.upper = fixedRange.lower + minSpan;
            if (fixedRange.upper > m_dataYMax) {
                fixedRange.upper = m_dataYMax;
                fixedRange.lower = std::max(m_dataYMin, fixedRange.upper - minSpan);
            }
        }
    } else {
        // Fallback: only prevent negative values
        if (fixedRange.lower < 0) fixedRange.lower = 0;
    }

    // Apply changes only if range was modified
    if (!qFuzzyCompare((float)fixedRange.lower, (float)newRange.lower) ||
        !qFuzzyCompare((float)fixedRange.upper, (float)newRange.upper)) {
        ui->customPlot->yAxis->setRange(fixedRange);
    }
}

/**
 * @brief Enforces positive X-axis range and data boundaries
 *
 * Prevents negative X-values and maintains minimum span for visibility.
 * Connected to X-axis rangeChanged signal.
 *
 * @param newRange The proposed new axis range
 */
void MainWindow::enforcePositiveXAxis(const QCPRange &newRange)
{
    QCPRange fixedRange = newRange;

    if (m_hasDataRange) {
        if (fixedRange.lower < m_dataXMin) fixedRange.lower = m_dataXMin;
        if (fixedRange.upper > m_dataXMax) fixedRange.upper = m_dataXMax;

        double span = fixedRange.upper - fixedRange.lower;
        double minSpan = std::max(1e-6, (m_dataXMax - m_dataXMin) * 0.0001);
        if (span < minSpan) {
            fixedRange.upper = fixedRange.lower + minSpan;
            if (fixedRange.upper > m_dataXMax) {
                fixedRange.upper = m_dataXMax;
                fixedRange.lower = std::max(m_dataXMin, fixedRange.upper - minSpan);
            }
        }
    } else {
        if (fixedRange.lower < 0) fixedRange.lower = 0;
    }

    if (!qFuzzyCompare((float)fixedRange.lower, (float)newRange.lower) ||
        !qFuzzyCompare((float)fixedRange.upper, (float)newRange.upper)) {
        ui->customPlot->xAxis->setRange(fixedRange);
    }
}

//=============================================================================
// PROCESS CONTROL
//=============================================================================

/**
 * @brief Stops the running monitoring process and cleans up resources
 *
 * Safely terminates the monitoring process, closes the real-time window,
 * and performs proper cleanup of all related resources.
 */
void MainWindow::on_stopButton_clicked()
{
    // Handle case where no process is running
    if (!runningProcess) {
        if (realtimeWindow) {
            disconnect(realtimeWindow, &RealtimeWindow::stopRequested,
                       this, &MainWindow::on_stopButton_clicked);
            realtimeWindow->stopUpdates();
            realtimeWindow->close();
            realtimeWindow->deleteLater();
            realtimeWindow = nullptr;
        }
        return;
    }

    // Keep local reference to avoid race conditions
    QProcess *proc = runningProcess;

    // Terminate process
    proc->kill();
    proc->waitForFinished();

    // Clean up real-time window
    if (realtimeWindow) {
        disconnect(realtimeWindow, &RealtimeWindow::stopRequested,
                   this, &MainWindow::on_stopButton_clicked);
        realtimeWindow->stopUpdates();
        realtimeWindow->close();
        realtimeWindow->deleteLater();
        realtimeWindow = nullptr;
    }

    // Schedule process deletion through event loop
    proc->deleteLater();
    if (proc == runningProcess) runningProcess = nullptr;

    // Load archive data if specified in UI
    if (!ui->archiveEdit->text().isEmpty())
        loadArchiveFromPath(ui->archiveEdit->text());
}

//=============================================================================
// UI EVENT HANDLERS
//=============================================================================

/**
 * @brief Handles changes to the device type selection (USO combobox)
 *
 * Enables/disables relevant input fields and updates the subdomain list
 * based on the selected device types (CPU/GPU).
 *
 * @param text The new selection text
 */
void MainWindow::onUseBoxChanged(const QString &text)
{
    QString uso = text.trimmed().toUpper();

    // Enable/disable input fields based on selection
    ui->subdomainEdit->setEnabled(uso.contains("CPU"));
    ui->gpuEdit->setEnabled(uso.contains("GPU"));

    // Rebuild the device list to reflect new selection
    updateListViewFromUseAndConfig();
}

/**
 * @brief Clears all UI fields and resets configuration state
 */
void MainWindow::on_clearButton_clicked()
{
    // Clear all text fields in the UI
    ui->lineEdit->clear();
    ui->sampleTimeEdit->clear();
    ui->postExecEdit->clear();
    ui->minPowerEdit->clear();
    ui->outputFIleEdit->clear();
    ui->subdomainEdit->clear();
    ui->gpuEdit->clear();
    ui->executableEdit->clear();
    ui->argumentsEdit->clear();

    // Reset internal configuration data
    currentConfigPath.clear();
    currentConfig.clear();
}

/**
 * @brief Periodic UI update slot for real-time data visualization
 *
 * Called by dataTimer to refresh the main plot with current data.
 * Maintains lightweight real-time preview capability.
 */
void MainWindow::realtimeDataSlot()
{
    if (subdomainModel->rowCount() == 0)
        return;

    ui->customPlot->clearGraphs();

    bool showEnergy = ui->energyCheck->isChecked();
    bool showPower  = ui->powerCheck->isChecked();

    // Recreate graphs for checked items
    for (int row = 0; row < subdomainModel->rowCount(); ++row) {
        QStandardItem *item = subdomainModel->item(row);
        if (item->checkState() == Qt::Checked) {
            QString key = item->text();
            // Get correct ID from UserRole or fallback
            QString id = item->data(Qt::UserRole).toString();
            if (id.isEmpty()) id = key;

            QVector<double> x = allTimestamps[key];
            QVector<double> y;

            if (showEnergy)
                y = accumulatedEnergy[key];
            else if (showPower)
                y = allPowerData[key];

            if (x.isEmpty() || y.isEmpty())
                continue;

            QCPGraph *graph = ui->customPlot->addGraph();
            graph->setName(key);
            graph->setData(x, y);

            // Use color from unified color map
            QColor color = deviceColorMap.value(id);
            graph->setPen(QPen(color, 2));
        }
    }

    ui->customPlot->replot();
}

//=============================================================================
// DEVICE LIST MANAGEMENT
//=============================================================================

/**
 * @brief Rebuilds the subdomain list model based on current configuration
 *
 * Creates checkable list items for CPU subdomains and GPU devices
 * according to the current USO selection and configuration values.
 * Provides fallback to detected data keys if no configuration is present.
 */
void MainWindow::updateListViewFromUseAndConfig()
{
    subdomainModel->clear();

    QString uso = ui->useBox->currentText().trimmed().toUpper();
    QStringList cpuSubdomains = ui->subdomainEdit->text().split(",", Qt::SkipEmptyParts);
    QStringList gpuIds = ui->gpuEdit->text().split(",", Qt::SkipEmptyParts);

    QSet<QString> added;

    // Helper lambda for creating list items
    auto createItem = [&](const QString &displayText, const QString &id) {
        if (displayText.isEmpty() || id.isEmpty()) return;
        if (added.contains(id)) return;

        QStandardItem *item = new QStandardItem(displayText);
        item->setEditable(false);
        item->setCheckable(true);
        item->setCheckState(Qt::Checked);
        item->setData(id, Qt::UserRole);   // Store stable ID in UserRole
        subdomainModel->appendRow(item);
        added.insert(id);

        // Assign color only when creating (persistent in deviceColorMap)
        ensureColorForId(id);
    };

    // Add CPU subdomains based on configuration
    if (uso.contains("CPU")) {
        for (const QString &sd : cpuSubdomains) {
            QString name = sd.trimmed();
            if (name.isEmpty()) continue;

            // Assume the key used in allTimestamps is exactly 'name'
            QString id = name;
            createItem(name, id);
        }
    }

    // Add GPU devices based on configuration
    if (uso.contains("GPU")) {
        for (const QString &idStr : gpuIds) {
            QString trimmed = idStr.trimmed();
            if (trimmed.isEmpty()) continue;
            QString display = QString("GPU%1").arg(trimmed);
            QString id = display; // Assume key = "GPU0", "GPU1", etc.
            createItem(display, id);
        }
    }

    // Fallback: if no items, use internal keys (ensure UserRole = key)
    if (subdomainModel->rowCount() == 0) {
        for (auto it = allEnergyData.constBegin(); it != allEnergyData.constEnd(); ++it) {
            QString id = it.key();
            createItem(id, id);
        }
    }

    updatePlotForSelection();
}

//=============================================================================
// FILE PARSING AND DATA PROCESSING
//=============================================================================

/**
 * @brief Loads and parses archive data from specified file path
 *
 * Processes output files containing power monitoring data, extracts
 * header information, parses data rows, computes cumulative energy,
 * and updates the visualization system.
 *
 * @param filePath Path to the archive data file
 */
void MainWindow::loadArchiveFromPath(const QString &filePath)
{
    QFile file(filePath);
    if (!file.exists()) {
        QMessageBox::warning(this, tr("File not found"),
                             tr("Could not open the output file:\n%1").arg(filePath));
        return;
    }
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
        QMessageBox::warning(this, tr("Error"),
                             tr("Failed to open the output file:\n%1").arg(filePath));
        return;
    }

    // Show selected path in UI
    ui->archiveEdit->setText(filePath);

    // Read the whole file into memory as lines (trimmed)
    QTextStream in(&file);
    QStringList lines;
    while (!in.atEnd()) {
        QString l = in.readLine().trimmed();
        if (!l.isEmpty())
            lines << l;
    }
    file.close();

    if (lines.isEmpty()) {
        QMessageBox::information(this, tr("Empty file"), tr("The output file is empty."));
        return;
    }

    // 1) Extract possible header information: SUBDOMAIN, USO, GPU
    QStringList cpuSubdomains;
    QString usoValue;
    QString gpuHeaderValue;
    for (const QString &line : lines) {
        if (line.startsWith("SUBDOMAIN=", Qt::CaseSensitive)) {
            cpuSubdomains = line.mid(QString("SUBDOMAIN=").length()).split(",", Qt::SkipEmptyParts);
        } else if (line.startsWith("USO=", Qt::CaseSensitive)) {
            usoValue = line.mid(QString("USO=").length()).trimmed().toUpper();
        } else if (line.startsWith("GPU=", Qt::CaseSensitive)) {
            gpuHeaderValue = line.mid(QString("GPU=").length()).trimmed();
        }
    }

    // 2) Clear previous data
    subdomainModel->clear();
    allTimestamps.clear();
    allEnergyData.clear();
    allPowerData.clear();
    accumulatedEnergy.clear();

    ui->customPlot->clearGraphs();
    deviceGraphs.clear();
    deviceColors.clear();

    // 3) Parse rows after a line starting with "timestamp" (case-insensitive)
    QSet<QString> gpuIdsSet;
    QSet<QString> cpuIdsSet;
    bool headerFound = false;
    for (const QString &line : lines) {
        if (line.startsWith("timestamp", Qt::CaseInsensitive)) {
            headerFound = true;
            continue;
        }
        if (!headerFound) continue; // ignore anything before the timestamp header

        QStringList parts = line.split(";", Qt::KeepEmptyParts);
        if (parts.size() < 5) continue;

        bool ok;
        double t = parts[0].toDouble(&ok);
        if (!ok) continue;

        QString device = parts[1].trimmed();
        QString id = parts[2].trimmed();
        double e = parts[3].toDouble(&ok); if (!ok) e = 0.0;
        double p = parts[4].toDouble(&ok); if (!ok) p = 0.0;

        QString key;
        if (device.compare("CPU", Qt::CaseInsensitive) == 0) {
            key = id;                  // CPU key e.g. "0-dram" (according to format)
            cpuIdsSet.insert(id);
        } else {
            key = QString("GPU%1").arg(id); // GPU -> "GPU0"
            gpuIdsSet.insert(id);
        }

        // Append parsed values to per-key vectors
        allTimestamps[key].append(t);
        allEnergyData[key].append(e);
        allPowerData[key].append(p);
    }

    // 4) Compute cumulative energy per key (accumulatedEnergy)
    for (auto it = allEnergyData.constBegin(); it != allEnergyData.constEnd(); ++it) {
        const QString &key = it.key();
        const QVector<double> &raw = it.value();
        QVector<double> acc;
        double sum = 0.0;
        for (double v : raw) {
            sum += v;
            acc.append(sum);
        }
        accumulatedEnergy[key] = acc;
    }

    // 5) Determine usoValue (if not provided in header) from detected keys
    if (usoValue.isEmpty()) {
        bool hasCPU = !cpuIdsSet.isEmpty();
        bool hasGPU = !gpuIdsSet.isEmpty();
        if (hasCPU && hasGPU) usoValue = "CPU,GPU";
        else if (hasCPU) usoValue = "CPU";
        else if (hasGPU) usoValue = "GPU";
    }

    // Helper: try to find the real KEY in allEnergyData from a header token
    auto findMatchingKey = [&](const QString &token) -> QString {
        if (token.isEmpty()) return QString();
        if (allEnergyData.contains(token)) return token;
        QString lowToken = token.toLower();
        for (auto it = allEnergyData.constBegin(); it != allEnergyData.constEnd(); ++it) {
            QString k = it.key();
            QString lowK = k.toLower();
            if (lowK == lowToken) return k;
            if (lowK.contains(lowToken)) return k;
            if (lowToken.contains(lowK)) return k;
        }
        return QString();
    };

    // 6) Build the listView items (prefer SUBDOMAIN header values for CPUs)
    QSet<QString> addedItems; // store keys (real ids) here

    if (usoValue.contains("CPU")) {
        // First try to use SUBDOMAIN= header (mapping to real keys if possible)
        for (const QString &sd : cpuSubdomains) {
            QString name = sd.trimmed();
            if (name.isEmpty()) continue;

            QString mappedKey = findMatchingKey(name);
            QString itemDisplay = name;
            QString idToUse;

            if (!mappedKey.isEmpty()) {
                idToUse = mappedKey;
            } else {
                // if no mapping found, try to use exactly the token
                idToUse = name;
            }

            if (idToUse.isEmpty() || addedItems.contains(idToUse)) continue;

            QStandardItem *item = new QStandardItem(itemDisplay);
            item->setEditable(false);
            item->setCheckable(true);
            item->setCheckState(Qt::Checked);
            item->setData(idToUse, Qt::UserRole); // <-- store the real KEY
            subdomainModel->appendRow(item);
            addedItems.insert(idToUse);

            ensureColorForId(idToUse);
        }

        // If no SUBDOMAIN header items were added, add detected CPU keys (raw)
        if (addedItems.isEmpty()) {
            for (const QString &id : cpuIdsSet) {
                QString k = id;
                if (addedItems.contains(k)) continue;
                QStandardItem *item = new QStandardItem(k); // k = real KEY used to populate allTimestamps
                item->setEditable(false);
                item->setCheckable(true);
                item->setCheckState(Qt::Checked);
                item->setData(k, Qt::UserRole); // <-- ESSENTIAL: store the real KEY
                subdomainModel->appendRow(item);
                addedItems.insert(k);
                ensureColorForId(k);
            }
        }
    }

    // GPUs: prefer detected ids, otherwise use GPU= header
    QStringList gpuIdsList;
    if (!gpuIdsSet.isEmpty()) {
        for (const QString &g : gpuIdsSet) gpuIdsList << g;
    } else if (!gpuHeaderValue.isEmpty()) {
        gpuIdsList = gpuHeaderValue.split(",", Qt::SkipEmptyParts);
    }

    if (usoValue.contains("GPU")) {
        for (const QString &id : gpuIdsList) {
            QString trimmed = id.trimmed();
            if (trimmed.isEmpty()) continue;
            QString gpuName = QString("GPU%1").arg(trimmed);
            if (addedItems.contains(gpuName)) continue;

            QStandardItem *item = new QStandardItem(gpuName);
            item->setEditable(false);
            item->setCheckable(true);
            item->setCheckState(Qt::Checked);

            // Store the real ID that matches the keys used in allTimestamps: "GPU0", "GPU1", ...
            item->setData(gpuName, Qt::UserRole);
            subdomainModel->appendRow(item);
            addedItems.insert(gpuName);
            ensureColorForId(gpuName);
        }
    }

    // Fallback: if still empty, add every key found in allEnergyData
    if (subdomainModel->rowCount() == 0) {
        for (auto it = allEnergyData.constBegin(); it != allEnergyData.constEnd(); ++it) {
            QString key = it.key();
            if (addedItems.contains(key)) continue;
            QStandardItem *item = new QStandardItem(key);
            item->setEditable(false);
            item->setCheckable(true);
            item->setCheckState(Qt::Checked);
            item->setData(key, Qt::UserRole); // stable ID

            ensureColorForId(key); // <--- here assign the fixed color
            subdomainModel->appendRow(item);
        }
    }

    // 7) Clear any leftover graphs/color maps and draw new ones based on parsed data
    ui->customPlot->clearGraphs();
    deviceGraphs.clear();
    deviceColors.clear();

    // Finally draw the selected ones using updatePlotForSelection()
    updatePlotForSelection();
}

//=============================================================================
// UTILITY FUNCTIONS
//=============================================================================

/**
 * @brief Retrieves device list from current configuration
 *
 * Constructs a list of device identifiers for real-time monitoring
 * based on the current USO selection and configuration values.
 *
 * @return QStringList of device identifiers
 */
QStringList MainWindow::getDevicesFromConfig() const
{
    QString uso = ui->useBox->currentText().trimmed().toUpper();

    QStringList devices;

    QStringList cpuSubdomains = ui->subdomainEdit->text().split(",", Qt::SkipEmptyParts);
    QStringList gpuIds = ui->gpuEdit->text().split(",", Qt::SkipEmptyParts);

    if (uso.contains("CPU")) {
        for (const QString &sd : cpuSubdomains) {
            QString name = sd.trimmed();
            if (!name.isEmpty())
                devices << name;  // each CPU subdomain as-is
        }
    }

    if (uso.contains("GPU")) {
        for (const QString &id : gpuIds) {
            QString trimmed = id.trimmed();
            if (!trimmed.isEmpty())
                devices << "GPU" + trimmed;  // GPU0, GPU1...
        }
    }

    return devices;
}

/**
 * @brief Validates RAPL subdomains against system configuration
 *
 * Checks specified RAPL subdomains against available system domains
 * by examining the /sys/class/powercap/intel-rapl directory structure.
 * Supports multiple naming formats for domain compatibility.
 *
 * @param subdomainText Comma-separated list of subdomain names to validate
 * @param failedToken Output parameter for the first invalid token found
 * @param reason Output parameter for detailed failure reason
 * @return true if all subdomains are valid, false otherwise
 */
bool MainWindow::validateSubdomains(const QString &subdomainText, QString &failedToken, QString &reason) const
{
    // Trim and split by comma
    QStringList tokens = subdomainText.split(",", Qt::SkipEmptyParts);
    for (QString &t : tokens) t = t.trimmed();
    if (tokens.isEmpty()) {
        reason = "No subdomains specified";
        return true; // Empty is valid
    }

    // Check sysfs root
    QDir raplRoot("/sys/class/powercap/intel-rapl");
    if (!raplRoot.exists()) {
        reason = QString("/sys/class/powercap/intel-rapl not found on this system");
        failedToken = tokens.isEmpty() ? QString() : tokens.first();
        return false;
    }

    // Build a comprehensive map of available RAPL domains with ALL possible name formats
    QMap<QString, QString> domainMap; // domain_name -> full_path

    // Get all RAPL entries
    QStringList allEntries = raplRoot.entryList(QDir::Dirs | QDir::NoDotAndDotDot);

    for (const QString &entry : allEntries) {
        if (!entry.startsWith("intel-rapl:")) continue;

        QString fullPath = raplRoot.absoluteFilePath(entry);

        // Extract CPU index from entry name (e.g., "intel-rapl:0" -> 0)
        QStringList parts = entry.split(':');
        if (parts.size() < 2) continue;
        QString cpuIndex = parts[1];

        // Read the domain name
        QFile nameFile(fullPath + "/name");
        if (nameFile.exists() && nameFile.open(QIODevice::ReadOnly | QIODevice::Text)) {
            QString domainName = QString(nameFile.readAll()).trimmed();
            nameFile.close();

            if (!domainName.isEmpty()) {
                // Add ALL possible naming formats for this domain

                // Format 1: "0-package" (CPU-SUBDOMAIN)
                QString cpuPrefixed = cpuIndex + "-" + domainName.split('-')[0];
                domainMap[cpuPrefixed] = fullPath;

                // Format 2: "package-0" (original name)
                domainMap[domainName] = fullPath;

                // Format 3: Just "package" (simple name)
                QString simpleName = domainName.split('-')[0];
                domainMap[simpleName] = fullPath;

                // Format 4: Full name with CPU prefix "0-package-0"
                QString fullPrefixed = cpuIndex + "-" + domainName;
                domainMap[fullPrefixed] = fullPath;
            }
        }

        // Check for subdomains (like intel-rapl:X:Y)
        QDir domainDir(fullPath);
        QStringList subEntries = domainDir.entryList(QDir::Dirs | QDir::NoDotAndDotDot);

        for (const QString &subEntry : subEntries) {
            if (subEntry.startsWith("intel-rapl:")) {
                QString subFullPath = domainDir.absoluteFilePath(subEntry);
                QFile subNameFile(subFullPath + "/name");

                if (subNameFile.exists() && subNameFile.open(QIODevice::ReadOnly | QIODevice::Text)) {
                    QString subDomainName = QString(subNameFile.readAll()).trimmed();
                    subNameFile.close();

                    if (!subDomainName.isEmpty()) {
                        // Add ALL possible naming formats for this subdomain

                        // Format 1: "0-dram" (CPU-SUBDOMAIN)
                        QString subCpuPrefixed = cpuIndex + "-" + subDomainName.split('-')[0];
                        domainMap[subCpuPrefixed] = subFullPath;

                        // Format 2: "dram-0" (original name)
                        domainMap[subDomainName] = subFullPath;

                        // Format 3: Just "dram" (simple name)
                        QString subSimpleName = subDomainName.split('-')[0];
                        domainMap[subSimpleName] = subFullPath;

                        // Format 4: Full name with CPU prefix "0-dram-0"
                        QString subFullPrefixed = cpuIndex + "-" + subDomainName;
                        domainMap[subFullPrefixed] = subFullPath;
                    }
                }
            }
        }
    }

    // Create a list of available domains for error messages
    QStringList availableDomains = domainMap.keys();
    availableDomains.sort();

    // Validate each token - ONLY EXACT MATCHES ALLOWED
    for (const QString &token : tokens) {
        if (token.isEmpty()) continue;

        // Check for EXACT match only (case-sensitive)
        if (!domainMap.contains(token)) {
            failedToken = token;
            reason = QString("Subdomain '%1' not found.\n\nAvailable domains:\n%2")
                        .arg(token)
                        .arg(availableDomains.join("\n"));
            return false;
        }

        qDebug() << "Subdomain" << token << "validated successfully";
    }

    return true;
}

/**
 * @brief Debug function to display available RAPL domains and naming formats
 *
 * Scans the system for RAPL domains and prints detailed information about
 * available domains, their naming formats, and validation test results.
 */
void MainWindow::debugRAPLDomains() {
    QDir raplRoot("/sys/class/powercap/intel-rapl");
    if (!raplRoot.exists()) {
        qDebug() << "RAPL root not found:" << raplRoot.absolutePath();
        return;
    }

    qDebug() << "=== RAPL Domains Detailed Debug ===";
    qDebug() << "RAPL root:" << raplRoot.absolutePath();

    QStringList entries = raplRoot.entryList(QDir::Dirs | QDir::NoDotAndDotDot);
    qDebug() << "All entries:" << entries;

    // Build the same domain map as validateSubdomains for consistency
    QMap<QString, QString> domainMap;

    int domainCount = 0;

    for (const QString &entry : entries) {
        if (!entry.startsWith("intel-rapl:")) {
            qDebug() << "Skipping non-RAPL entry:" << entry;
            continue;
        }

        QString fullPath = raplRoot.absoluteFilePath(entry);
        qDebug() << "=== RAPL Domain:" << entry << "===";
        qDebug() << "Full path:" << fullPath;

        // Extract CPU index
        QStringList parts = entry.split(':');
        QString cpuIndex = parts.size() >= 2 ? parts[1] : "unknown";

        // Read name file
        QFile nameFile(fullPath + "/name");
        if (nameFile.exists() && nameFile.open(QIODevice::ReadOnly | QIODevice::Text)) {
            QString name = QString(nameFile.readAll()).trimmed();
            nameFile.close();
            qDebug() << "Original name:" << name;

            // Show all accepted formats
            QString simpleName = name.split('-')[0];
            qDebug() << "Accepted formats:";
            qDebug() << "  -" << (cpuIndex + "-" + simpleName); // 0-package
            qDebug() << "  -" << name; // package-0
            qDebug() << "  -" << simpleName; // package
            qDebug() << "  -" << (cpuIndex + "-" + name); // 0-package-0

            // Add to domain map for testing
            domainMap[cpuIndex + "-" + simpleName] = fullPath;
            domainMap[name] = fullPath;
            domainMap[simpleName] = fullPath;
            domainMap[cpuIndex + "-" + name] = fullPath;
        } else {
            qDebug() << "Name file: missing or unreadable";
        }

        // Check subdomains
        QDir domainDir(fullPath);
        QStringList subEntries = domainDir.entryList(QDir::Dirs | QDir::NoDotAndDotDot);
        qDebug() << "Subdomains:" << subEntries;

        for (const QString &subEntry : subEntries) {
            if (subEntry.startsWith("intel-rapl:")) {
                QString subFullPath = domainDir.absoluteFilePath(subEntry);
                QFile subNameFile(subFullPath + "/name");
                if (subNameFile.exists() && subNameFile.open(QIODevice::ReadOnly | QIODevice::Text)) {
                    QString subName = QString(subNameFile.readAll()).trimmed();
                    subNameFile.close();
                    qDebug() << "  Subdomain" << subEntry << "-> Original name:" << subName;

                    QString subSimpleName = subName.split('-')[0];
                    qDebug() << "  Accepted formats:";
                    qDebug() << "    -" << (cpuIndex + "-" + subSimpleName); // 0-dram
                    qDebug() << "    -" << subName; // dram-0
                    qDebug() << "    -" << subSimpleName; // dram
                    qDebug() << "    -" << (cpuIndex + "-" + subName); // 0-dram-0

                    // Add to domain map for testing
                    domainMap[cpuIndex + "-" + subSimpleName] = subFullPath;
                    domainMap[subName] = subFullPath;
                    domainMap[subSimpleName] = subFullPath;
                    domainMap[cpuIndex + "-" + subName] = subFullPath;
                }
            }
        }
        domainCount++;
        qDebug() << ""; // Empty line for separation
    }

    qDebug() << "Total RAPL domains found:" << domainCount;

    // Show all accepted formats
    QStringList allAcceptedFormats = domainMap.keys();
    allAcceptedFormats.sort();
    qDebug() << "All accepted formats:" << allAcceptedFormats;

    qDebug() << "=== End RAPL Debug ===";

    // Test validation with various patterns - including invalid ones
    QStringList testPatterns = {
        "0-package", "package-0", "package", "0-package-0",  // Valid
        "0-dram", "dram-0", "dram", "0-dram-0",              // Valid
        "1", "2-psys", "1-fdsfsfg", "asfdfadfs",             // Invalid
        "0-core", "core-0", "core", "0-core-0"               // Valid if exists
    };

    for (const QString &pattern : testPatterns) {
        QString failedToken, reason;
        bool isValid = validateSubdomains(pattern, failedToken, reason);
        qDebug() << "Test pattern '" << pattern << "' ->" << (isValid ? "VALID" : "INVALID");
        if (!isValid) {
            qDebug() << "  Reason:" << reason;
        }
    }
}

//=============================================================================
// UI EVENT HANDLERS (CONTINUED)
//=============================================================================

/**
 * @brief Displays application information dialog
 */
void MainWindow::on_aboutButton_clicked()
{
    QString aboutText =
        "Application for power and energy measurement on CPU+GPU platforms\n\n"
        "Author: José Javier Pérez Sánchez\n"
        "Work supervised by: Sergio Santander Jiménez and José María Granado Criado\n\n"
        "© 2025 - All rights reserved.";

    QMessageBox::information(
        this,
        tr("About This Application"),
        aboutText
    );
}

/**
 * @brief Handles plot double-click event to reset view to data boundaries
 */
void MainWindow::onCustomPlotDoubleClick()
{
    if (!m_hasDataRange) return;

    ui->customPlot->xAxis->setRange(m_dataXMin, m_dataXMax);
    ui->customPlot->yAxis->setRange(m_dataYMin, m_dataYMax);
    ui->customPlot->replot();
}

/**
 * @brief Placeholder for plot zoom/pan change events
 */
void MainWindow::onPlotZoomOrPanChanged() {}

/**
 * @brief Handles plot double-click to reset to original ranges
 */
void MainWindow::onPlotDoubleClick()
{
    // Reset to original graph limits
    ui->customPlot->xAxis->setRange(originalXMin, originalXMax);
    ui->customPlot->yAxis->setRange(originalYMin, originalYMax);
    ui->customPlot->replot();
}

//=============================================================================
// COLOR MANAGEMENT
//=============================================================================

/**
 * @brief Ensures consistent color assignment for device identifiers
 *
 * Assigns colors from a predefined palette to device IDs, recycling colors
 * when the palette is exhausted to maintain consistency across sessions.
 *
 * @param id Device identifier to assign color to
 */
void MainWindow::ensureColorForId(const QString &id)
{
    if (deviceColorMap.contains(id)) return;

    // Exact palette in specified order
    static QVector<QColor> palette = {
        QColor(31, 119, 180),   // Blue
        QColor(255, 127, 14),   // Orange
        QColor(44, 160, 44),    // Green
        QColor(214, 39, 40),    // Red
        QColor(148, 103, 189),  // Purple
        QColor(140, 86, 75),    // Brown
        QColor(227, 119, 194),  // Pink
        QColor(127, 127, 127),  // Gray
        QColor(188, 189, 34),   // Lime
        QColor(23, 190, 207)    // Cyan
    };

    // If colors run out, recycle from the beginning
    int nextIndex = deviceColorMap.size() % palette.size();

    deviceColorMap[id] = palette[nextIndex];

    qDebug() << "Asignado color para" << id << ":" << deviceColorMap[id].name() << "Índice:" << nextIndex;
}

/**
 * @brief Retrieves assigned color for device identifier
 *
 * @param deviceId Device identifier to get color for
 * @return QColor assigned to the device, or default gray if not found
 */
QColor MainWindow::getDeviceColor(const QString &deviceId) const {
    return deviceColorMap.value(deviceId, QColor(128, 128, 128)); // Gray by default
}
