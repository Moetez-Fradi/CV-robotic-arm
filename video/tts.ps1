# Text-to-speech with the voices installed in Windows (local, nothing leaves the machine).
# Usage: powershell -NoProfile -File tts.ps1 -Json items.json -Voice "Microsoft David Desktop" -Rate 0
# items.json: [{"path": "out.wav", "text": "..."}, ...]   Existing files are skipped.
param([string]$Json, [string]$Voice = "Microsoft David Desktop", [int]$Rate = 0)
Add-Type -AssemblyName System.Speech
$items = [System.IO.File]::ReadAllText($Json, [System.Text.Encoding]::UTF8) | ConvertFrom-Json
$synth = New-Object System.Speech.Synthesis.SpeechSynthesizer
$synth.SelectVoice($Voice)
$synth.Rate = $Rate
foreach ($it in $items) {
    if (Test-Path $it.path) { continue }
    $synth.SetOutputToWaveFile($it.path)
    $synth.Speak($it.text)
    $synth.SetOutputToNull()
}
$synth.Dispose()
