import { Component } from '@angular/core';

@Component({
  selector: 'app-download',
  templateUrl: './download.component.html',
  styleUrls: ['./download.component.scss']
})
export class DownloadComponent {
  constructor() { }

  download(): void {
    // Create a text file content
    const fileContent = 'This is a downloadable text file.';
    
    // Create a blob from the content
    const blob = new Blob([fileContent], { type: 'text/plain' });

    // Create a temporary URL for the blob
    const url = window.URL.createObjectURL(blob);

    // Create a link element
    const link = document.createElement('a');
    link.href = url;
    link.download = 'downloaded_file.txt'; // Set the file name

    // Append the link to the document body
    document.body.appendChild(link);

    // Click the link to trigger the download
    link.click();

    // Clean up
    window.URL.revokeObjectURL(url);
    link.remove();
  }
}
