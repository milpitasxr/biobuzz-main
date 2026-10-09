param(
    [ValidateRange(5,120)][int]$Seconds = 20,
    [ValidatePattern('^[a-zA-Z0-9_-]+$')][string]$Prefix = 'vision-live-sample'
)
$ErrorActionPreference = 'Stop'
# Read-only sampling: this script never sends INIT, START, or actuator commands.
$visionSocket = [System.Net.WebSockets.ClientWebSocket]::new()
$visionSocket.Options.KeepAliveInterval = [TimeSpan]::FromSeconds(1)
$visionTimeout = [System.Threading.CancellationTokenSource]::new(($Seconds + 15) * 1000)
$visionPackets = [System.Collections.Generic.List[object]]::new()
$visionResults = [System.Collections.Generic.List[object]]::new()
$visionSeen = [System.Collections.Generic.HashSet[string]]::new()
$visionWatch = [System.Diagnostics.Stopwatch]::new()
$visionLastStatus = -1.0
$visionSavedImage = $false
try {
    $null = $visionSocket.ConnectAsync([Uri]'ws://localhost:8000', $visionTimeout.Token).GetAwaiter().GetResult()
    $visionBuffer = [byte[]]::new(1048576)
    while (!$visionWatch.IsRunning -or $visionWatch.Elapsed.TotalSeconds -lt $Seconds) {
        # The Dashboard client normally requests status periodically; keep this reader alive too.
        if ($visionWatch.Elapsed.TotalSeconds - $visionLastStatus -ge 1) {
            $visionStatus = [Text.Encoding]::UTF8.GetBytes('{"type":"GET_ROBOT_STATUS"}')
            $null = $visionSocket.SendAsync([ArraySegment[byte]]::new($visionStatus), [System.Net.WebSockets.WebSocketMessageType]::Text, $true, $visionTimeout.Token).GetAwaiter().GetResult()
            $visionLastStatus = $visionWatch.Elapsed.TotalSeconds
        }
        $visionMessage = [IO.MemoryStream]::new()
        do {
            $visionReply = $visionSocket.ReceiveAsync([ArraySegment[byte]]::new($visionBuffer), $visionTimeout.Token).GetAwaiter().GetResult()
            if ($visionReply.MessageType -eq [System.Net.WebSockets.WebSocketMessageType]::Close) { throw 'Dashboard closed connection' }
            $visionMessage.Write($visionBuffer, 0, $visionReply.Count)
        } while (!$visionReply.EndOfMessage)
        $visionObject = [Text.Encoding]::UTF8.GetString($visionMessage.ToArray()) | ConvertFrom-Json
        $visionMessage.Dispose()
        if ($visionObject.type -eq 'RECEIVE_IMAGE' -and !$visionSavedImage -and $visionWatch.Elapsed.TotalSeconds -ge 5) {
            [IO.File]::WriteAllBytes((Join-Path $PWD "$Prefix.jpg"), [Convert]::FromBase64String($visionObject.imageString))
            $visionSavedImage = $true
        }
        if ($visionObject.type -ne 'RECEIVE_TELEMETRY') { continue }
        foreach ($visionPacket in $visionObject.telemetry) {
            $visionData = $visionPacket.data
            if ($visionData.'Detector error' -or $visionData.'Vision setup error') { throw ($visionData | ConvertTo-Json) }
            if ($visionData.Camera -ne 'STREAMING') { continue }
            if (!$visionWatch.IsRunning) { $visionWatch.Start() }
            $visionPackets.Add($visionData)
            # Dashboard repeats the latest result. Count each timing tuple only once.
            $visionKey = '{0}|{1}|{2}' -f $visionData.'Inference ms', $visionData.'Capture to result ms', $visionData.'Proposals ms'
            if ([double]$visionData.'Inference ms' -gt 0 -and $visionSeen.Add($visionKey)) { $visionResults.Add($visionData) }
        }
    }
} finally {
    $visionSocket.Dispose()
    $visionTimeout.Dispose()
}
if ($visionResults.Count -eq 0) { throw 'No completed vision results received' }
$visionPackets | ConvertTo-Json -Depth 5 | Set-Content -LiteralPath "$Prefix-telemetry.json"
$visionSummary = [ordered]@{
    seconds = $visionWatch.Elapsed.TotalSeconds
    packets = $visionPackets.Count
    uniqueObservedResults = $visionResults.Count
    precision = $visionResults[0].'CNN precision'
    workers = $visionResults[0].'Detector workers'
    note = 'Telemetry sampling can miss results faster than its publication interval; this is not a per-frame trace.'
}
foreach ($visionMetric in @('Inference ms', 'Capture to result ms', 'Proposals ms', 'CNN CPU sum ms', 'Crop CPU sum ms', 'CNN calls', 'Objects', 'Detection FPS', 'Camera FPS')) {
    $visionValues = @($visionResults | ForEach-Object { [double]$_.$visionMetric } | Sort-Object)
    $visionStats = $visionValues | Measure-Object -Average -Minimum -Maximum
    $visionSummary[$visionMetric] = [ordered]@{
        mean = $visionStats.Average
        min = $visionStats.Minimum
        max = $visionStats.Maximum
        p95 = $visionValues[[Math]::Floor(.95 * ($visionValues.Count - 1))]
    }
}
$visionSummary.objectsHistogram = @($visionResults | Group-Object Objects | ForEach-Object { @{objects=$_.Name; count=$_.Count} })
$visionSummary | ConvertTo-Json -Depth 5 | Set-Content -LiteralPath "$Prefix-summary.json"
$visionSummary | ConvertTo-Json -Depth 5
