param (
    [string]$sitePath = "./_site",
    [string]$repoUrl = "https://github.com/bepu/bepuphysics2/blob/master"
)

# Get all HTML files in the site, excluding those in the api/ directory
$htmlFiles = Get-ChildItem -Path $sitePath -Filter "*.html" -Recurse | Where-Object { $_.FullName -notmatch "\\api\\" }

$linkCount = 0
foreach ($file in $htmlFiles) {
    $content = Get-Content -Path $file.FullName -Raw
    $originalContent = $content
    
    Write-Host "Considering file: $($file.FullName)"
    
    # Find all links that end with .cs
    $pattern = '(href=["''])([^"'']*\.cs)(["''])'
    
    # Use a scriptblock for the replacement to keep the original quote style and only modify the URL
    $newContent = [regex]::Replace($content, $pattern, {
        param($match)
        $prefix = $match.Groups[1].Value  # href=" or href='
        $path = $match.Groups[2].Value    # path
        $suffix = $match.Groups[3].Value  # " or '
        
        # Skip if it's already a full URL
        if ($path -match '^https?:\/\/') {
            Write-Host "  Skipping already absolute URL: $path"
            return $match.Value
        }
        
        # If the path starts with "../", remove that prefix for the GitHub URL
        if ($path -match '^\.\.\/')  {
            $relativePath = $path -replace '^\.\.\/', ''
            Write-Host "  Rewriting link with ../ prefix: $path -> $repoUrl/$relativePath"
            return "$prefix$repoUrl/$relativePath$suffix"
        }
        
        # Otherwise, just prepend the GitHub URL to the relative path
        Write-Host "  Rewriting link ending with .cs: $path -> $repoUrl/$path"
        return "$prefix$repoUrl/$path$suffix"
    })
    
    # Only write the file if changes were made
    if ($newContent -ne $originalContent) {
        $matches = [regex]::Matches($content, $pattern)
        $linkCount += $matches.Count
        Set-Content -Path $file.FullName -Value $newContent
        Write-Host "Fixed $($matches.Count) links in $($file.Name)"
    }
}

Write-Host "Link transformation complete. Fixed $linkCount links in total."