import { Component, Input } from '@angular/core';
import { ScienceService } from 'src/app/service/science.service';

@Component({
  selector: 'app-download',
  templateUrl: './download.component.html',
  styleUrls: ['./download.component.scss']
})
export class DownloadComponent {
  @Input() type:string;

  constructor(private scienceService :ScienceService) {  }

  download(): void {
    // Create a text file content
    // const fileContent = 'This is a downloadable text file.';
    let fileContent:string;
    switch (this.type) {
      case "science":
        let data = this.scienceService.getStoredData();
        let headers = ["G1-4", "M5-8", "PH9-12"];
        fileContent = this.convertToCSV(headers, data);
        break;
      default:
        console.log("no data store for:" + this.type);
        fileContent = 'No data';
    }
  
    // Rest is okay change to csv
    // Create a blob from the content
    const blob = new Blob([fileContent], { type: 'text/csv' });

    // Create a temporary URL for the blob
    const url = window.URL.createObjectURL(blob);

    // Create a link element
    const link = document.createElement('a');
    link.href = url;
    link.download = 'data_file.csv' ; // Set the file name

    // Append the link to the document body
    document.body.appendChild(link);

    // Click the link to trigger the download
    link.click();

    // Clean up
    window.URL.revokeObjectURL(url);
    link.remove();
  }

  private convertToCSV(headers: string[], data: number[][]): string {
    const csvRows = [];
    csvRows.push(headers.join(','));
    for (const item of data) {
      csvRows.push(item.toString());
    }
    return csvRows.join('\n');
  }
}
